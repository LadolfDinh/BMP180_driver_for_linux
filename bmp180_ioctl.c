#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "bmp180_driver"
#define CLASS_NAME "bmp180"
#define DEVICE_NAME "bmp180"

#define BMP180_OSS 0x00

#define BMP180_REGISTER_AC1 0xAA
#define BMP180_REGISTER_AC2 0xAC
#define BMP180_REGISTER_AC3 0xAE
#define BMP180_REGISTER_AC4 0xB0
#define BMP180_REGISTER_AC5 0xB2
#define BMP180_REGISTER_AC6 0xB4
#define BMP180_REGISTER_B1  0xB6
#define BMP180_REGISTER_B2  0xB8
#define BMP180_REGISTER_MB  0xBA
#define BMP180_REGISTER_MC  0xBC
#define BMP180_REGISTER_MD  0xBE

#define BMP180_REG_CONTROL 0xF4
#define BMP180_REG_RESULT  0xF6    // register address

// IOCTL commands
#define BMP180_IOCTL_MAGIC 'm'
#define BMP180_IOCTL_READ_PRES _IOR(BMP180_IOCTL_MAGIC, 1, int)
#define BMP180_IOCTL_READ_TEMP _IOR(BMP180_IOCTL_MAGIC, 2, int)

static struct i2c_client *bmp180_client;
static struct class* bmp180_class = NULL;
static struct device* bmp180_device = NULL;
static int major_number;

static s16 AC1, AC2, AC3, B1, B2, MB, MC, MD, B5;
static u16 AC4, AC5, AC6;

static int bmp180_read_word(struct i2c_client *client, u8 reg)
{
    int msb = i2c_smbus_read_byte_data(client, reg);
    if (msb < 0) return msb;
    int lsb = i2c_smbus_read_byte_data(client, reg + 1);
    if (lsb < 0) return lsb;
    return (msb << 8) | lsb;
}

static int bmp180_read_calibration(struct i2c_client *client)
{
    AC1 = bmp180_read_word(client, BMP180_REGISTER_AC1);
    AC2 = bmp180_read_word(client, BMP180_REGISTER_AC2);
    AC3 = bmp180_read_word(client, BMP180_REGISTER_AC3);
    AC4 = bmp180_read_word(client, BMP180_REGISTER_AC4);
    AC5 = bmp180_read_word(client, BMP180_REGISTER_AC5);
    AC6 = bmp180_read_word(client, BMP180_REGISTER_AC6);
    B1  = bmp180_read_word(client, BMP180_REGISTER_B1);
    B2  = bmp180_read_word(client, BMP180_REGISTER_B2);
    MB  = bmp180_read_word(client, BMP180_REGISTER_MB);
    MC  = bmp180_read_word(client, BMP180_REGISTER_MC);
    MD  = bmp180_read_word(client, BMP180_REGISTER_MD);
    return 0;
}

static long bmp180_read_ut(struct i2c_client *client)
{
    int ret = i2c_smbus_write_byte_data(client, BMP180_REG_CONTROL, 0x2E);
    if (ret < 0) return ret;
    msleep(5);
    int msb = i2c_smbus_read_byte_data(client, BMP180_REG_RESULT);
    if (msb < 0) return msb;
    int lsb = i2c_smbus_read_byte_data(client, BMP180_REG_RESULT + 1);
    if (lsb < 0) return lsb;
    return (msb << 8) | lsb;
}

static long bmp180_read_up(struct i2c_client *client, int oss)
{
    int ret = i2c_smbus_write_byte_data(client, BMP180_REG_CONTROL, 0x34 + (oss << 6));
    if (ret < 0) return ret;
    msleep(2 + (3 << oss));

    int msb = i2c_smbus_read_byte_data(client, BMP180_REG_RESULT);
    if (msb < 0) return msb;
    int lsb = i2c_smbus_read_byte_data(client, BMP180_REG_RESULT + 1);
    if (lsb < 0) return lsb;
    int xlsb = i2c_smbus_read_byte_data(client, BMP180_REG_RESULT + 2);
    if (xlsb < 0) return xlsb;

    return (((msb << 16) | (lsb << 8) | xlsb) >> (8 - oss));
}

static long bmp180_calc_true_temp(long UT)
{
    s32 X1 = ((UT - (s32)AC6) * (s32)AC5) >> 15;
    s32 X2 = ((s32)MC << 11) / (X1 + (s32)MD);
    B5 = X1 + X2;
    return (B5 + 8) >> 4;
}

static long bmp180_calc_true_pressure(long UP, int oss)
{
    s32 B6 = B5 - 4000;
    s32 X1 = ((s32)B2 * ((B6 * B6) >> 12)) >> 11;
    s32 X2 = ((s32)AC2 * B6) >> 11;
    s32 X3 = X1 + X2;
    s32 B3 = ((((s32)AC1 * 4 + X3) << oss) + 2) >> 2;
    X1 = ((s32)AC3 * B6) >> 13;
    X2 = ((s32)B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    u32 B4 = ((u32)AC4 * (u32)(X3 + 32768)) >> 15;
    u32 B7 = ((u32)UP - B3) * (50000 >> oss);

    long p = (B7 < 0x80000000) ? (B7 << 1) / B4 : (B7 / B4) << 1;
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    return p + ((X1 + X2 + 3791) >> 4);
}

static long bmp180_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long data;
    long temp_ut, temp_up;

    switch (cmd) {
        case BMP180_IOCTL_READ_TEMP:
            temp_ut = bmp180_read_ut(bmp180_client);
            data = bmp180_calc_true_temp(temp_ut);
            break;
        case BMP180_IOCTL_READ_PRES:
            temp_ut = bmp180_read_ut(bmp180_client);
            bmp180_calc_true_temp(temp_ut);
            temp_up = bmp180_read_up(bmp180_client, BMP180_OSS);
            data = bmp180_calc_true_pressure(temp_up, BMP180_OSS);
            break;
        default:
            return -EINVAL;
    }

    if (copy_to_user((int __user *)arg, &data, sizeof(data)))
        return -EFAULT;

    return 0;
}

static int bmp180_open(struct inode *inodep, struct file *filep)
{
    printk(KERN_INFO "BMP180 device opened\n");
    return 0;
}

static int bmp180_release(struct inode *inodep, struct file *filep)
{
    printk(KERN_INFO "BMP180 device closed\n");
    return 0;
}

static struct file_operations fops = {
    .open = bmp180_open,
    .unlocked_ioctl = bmp180_ioctl,
    .release = bmp180_release,
};

static int bmp180_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    bmp180_client = client;
    bmp180_read_calibration(client);

    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_number < 0) return major_number;

    bmp180_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(bmp180_class)) {
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(bmp180_class);
    }

    bmp180_device = device_create(bmp180_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(bmp180_device)) {
        class_destroy(bmp180_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(bmp180_device);
    }

    printk(KERN_INFO "BMP180 driver installed\n");
    return 0;
}

static void bmp180_remove(struct i2c_client *client)
{
    device_destroy(bmp180_class, MKDEV(major_number, 0));
    class_unregister(bmp180_class);
    class_destroy(bmp180_class);
    unregister_chrdev(major_number, DEVICE_NAME);
    printk(KERN_INFO "BMP180 driver removed\n");
}

static const struct of_device_id bmp180_of_match[] = {
    { .compatible = "invensense,bmp180", },
    { },
};
MODULE_DEVICE_TABLE(of, bmp180_of_match);

static struct i2c_driver bmp180_driver = {
    .driver = {
        .name   = DRIVER_NAME,
        .owner  = THIS_MODULE,
        .of_match_table = of_match_ptr(bmp180_of_match),
    },
    .probe      = bmp180_probe,
    .remove     = bmp180_remove,
};

static int __init bmp180_init(void)
{
    printk(KERN_INFO "Initializing BMP180 driver\n");
    return i2c_add_driver(&bmp180_driver);
}

static void __exit bmp180_exit(void)
{
    printk(KERN_INFO "Exiting BMP180 driver\n");
    i2c_del_driver(&bmp180_driver);
}

module_init(bmp180_init);
module_exit(bmp180_exit);

MODULE_AUTHOR("Ladolf_Hitke");
MODULE_DESCRIPTION("BMP180 I2C Client Driver with IOCTL Interface");
MODULE_LICENSE("MIT");
