/* 
  C sample application that writes to the raspicomm's RS-485 Port  
*/

#include <stdio.h>   /* included for printf() */
#include <fcntl.h>   /* included for O_RDWR, O_NOCTTY and O_NDELAY */
#include <termios.h> /* included for functions and defines used in set_interface_attribs */
#include <unistd.h>  /* included for function write() */
#include <string.h>  /* included for memset */

int set_interface_attribs(int fd, int speed, int parity); 

#define RS485_DEVICE "/dev/ttyRPC0"
#define MESSAGE_TO_SEND "Hello World!"

/* main entry point */
int main(int argc, char** argv)
{
  int fd;
  int written_bytes;

  /* try to open the raspicomm rs-485 port */
  printf("this sample application writes to the rs-485 port\n");

  /* open the port */
  printf("opening device %s...", RS485_DEVICE);
  if ( (fd = open(RS485_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY)) < 0 )
  {
    printf("failed.\n");
    printf("possible causes:\n");
    printf("1) the raspicomm device driver is not loaded. type 'lsmod' and verify that you 'raspicommrs485' is loaded.\n");
    printf("2) the raspicomm device driver is in use. Is another application using the device driver?\n" );
    printf("3) something went wrong when loading the device driver. type 'dmesg' and check the kernel messages\n");
    return -1;
  }
  printf("successful. fd=%i\n", fd);

  /* configure the port */
  if (set_interface_attribs(fd, B9600, 0) < 0) {
    printf("error configuring the rs-485 port.\n");
    return -2;
  }

  /* example write */  
  printf("writing the message: (%i) %s to the rs-485 port\n", sizeof(MESSAGE_TO_SEND), MESSAGE_TO_SEND);
  written_bytes = write(fd, MESSAGE_TO_SEND, sizeof(MESSAGE_TO_SEND));

  printf("bytes written: %i\n", written_bytes);

  /* close the device again */
  printf("closing the device again\n");
  close(fd);
}

/* helper function to configure tty devices, see http://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c */
int set_interface_attribs (int fd, int speed, int parity)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
  {
    printf("error using tcgetattr\n");
    return -1;
  }

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // ignore break signal
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

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
  {
    printf("error from tcsetattr\n");
    return -1;
  }
  return 0;
}
