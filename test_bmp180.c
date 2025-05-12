#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h> // Include errno header

#define DEVICE_PATH "/dev/bmp180"
#define BMP180_IOCTL_MAGIC 'm'
#define BMP180_IOCTL_READ_PRES _IOR(BMP180_IOCTL_MAGIC, 1, int)
#define BMP180_IOCTL_READ_TEMP _IOR(BMP180_IOCTL_MAGIC, 2, int)

int main(void)
{
    int fd;
    long temp, pres;
//open
    fd = open(DEVICE_PATH, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open the device");
        return errno;
    }
// read
    if (ioctl(fd, BMP180_IOCTL_READ_PRES, &pres) < 0) {
        perror("Failed to read presure data");
        close(fd);
        return errno;
    }
    printf("Presure in Pa: %d\n", pres);

    if (ioctl(fd, BMP180_IOCTL_READ_TEMP, &temp) < 0) {
        perror("Failed to read temperture data");
        close(fd);
        return errno;
    }
    printf("Temperture: %d\n", temp);

    close(fd);
    return 0;

}



//
// Created by cudin on 12/05/2025.
//
