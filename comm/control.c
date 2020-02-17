#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

// @see https://stackoverflow.com/a/38318768/9827248

int set_interface_attribs(int fd, int speed, int parity)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0) {
    printf("Error from tcgetattr: %s\n", strerror(errno));
    return -1;
  }

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
                                  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if ( tcsetattr(fd, TCSANOW, &tty) != 0) {
    printf("Error from tcsetattr: %s\n", strerror(errno));
    return -1;
  }
  return 0;
}

void set_blocking (int fd, int should_block)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0) {
    printf("Error from tggetattr: %s\n", strerror(errno));
    return;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 50;

  if (tcsetattr (fd, TCSANOW, &tty) != 0) {
    printf("Error from tcsetattr: %s\n", strerror(errno));
    return;
  }
}

void write_serial(int fd, const char *str, int len)
{
 /* simple output */
  int wlen = write(fd, str, len);
  if (wlen != len) {
      printf("Error from write: %d, %d\n", wlen, errno);
  }
  tcdrain(fd);    /* delay for output */
}

void read_serial(int fd) {
  unsigned char buf[80];

  int rdlen = read(fd, buf, sizeof(buf) - 1);
  if (rdlen > 0) {
      unsigned char *p;
      printf("Read %d:", rdlen);
      for (p = buf; rdlen-- > 0; p++) {
          printf(" 0x%x", *p);
      }
      printf("\n");

  } else if (rdlen < 0) {
      printf("Error from read: %d: %s\n", rdlen, strerror(errno));
  } else {  /* rdlen == 0 */
      printf("Timeout from read\n");
  }               
}

int main(int argc, char *argv[]) {

  if (argc < 2) {
    printf("No device specified\n");
    return -1;
  }

  if (argc < 3) {
    printf("No value specified\n");
    return -1;
  }

  int fd = open(argv[1], O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
      printf("Error opening %s: %s\n", argv[1], strerror(errno));
      return -1;
  }
  
/*
See HW hack to avoid reset https://tushev.org/articles/arduino/22/preventing-arduino-from-auto-reset-when-com-port-opens-closes
  #include <sys/ioctl.h>

  int serial;
  ioctl(fd, TIOCMGET, &serial);
  if (serial & TIOCM_DTR)
    puts("TIOCM_DTR is set");
  else
    puts("TIOCM_DTR is not set");

  int iflags = TIOCM_DTR;
  //ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
  ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR
*/

  set_interface_attribs(fd, B9600, 0);
  set_blocking(fd, 0);

  usleep(2 * 1000 * 1000); 
  
  unsigned char buf[16];
  memset (&buf, 0, sizeof buf);

  buf[0] = atoi(argv[2]);

  write_serial(fd, buf, 1);
  read_serial(fd);

  return 0;
}