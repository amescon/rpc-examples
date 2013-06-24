/* 
  C raspicomm sample application that uses the poll() function to wait for data / stdin (keyboard) events
  by using poll() we're not spinning the cpu while waiting for data 
  and we're able to respond to other events, should they happen
*/

#include <stdio.h>   /* included for printf() */
#include <fcntl.h>   /* included for O_RDWR, O_NOCTTY and O_NDELAY */
#include <termios.h> /* included for functions and defines used in set_interface_attribs */
#include <unistd.h>  /* included for function write() */
#include <string.h>  /* included for memset */

#include <poll.h>    /* included for poll() */

int set_interface_attribs(int fd, int speed, int parity); 

#define RS485_DEVICE "/dev/ttyRPC0"
#define READ_BUFFER_SIZE 10

static void enter_poll_loop(int fd);

/* functions used to configure stdin to not wait for a linefeed */
static void initTermios(int echo);
static void restoreTermios();
static struct termios _old, _new;

/* main entry point */
int main(int argc, char** argv)
{
  int fd, readBytes, i, fd_stdin; char readBuffer[READ_BUFFER_SIZE], data;
  
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

  /* enter the polling loop */
  enter_poll_loop(fd); 

  /* close the device again */
  close(fd);  
}

/* uses poll() to wait for data reception */
static void enter_poll_loop(int fd)
{
  int numReady, stopPolling = 0;
  short pollevent = POLLIN;
  const int fdCount = 2;
  struct pollfd fds[fdCount];
  fds[0].fd = fd;
  fds[0].events = pollevent;
  
  fds[1].fd = STDIN_FILENO;
  fds[1].events = pollevent;

  initTermios(0);

  char data;
  int readBytes;

  int readCount = 0;
  int maxReadCount = 10;

  printf("use poll() to check if data is available from the rs-485 port or a key was pressed.\n");
  printf("quits after %i bytes are received or the 'q' key is pressed\n", maxReadCount);

  do 
  {
    /* wait an unlimited time for something to happen (e.g. data from the rs-485 port or keyboard input) */
    numReady = poll(fds, fdCount, -1);

    if (numReady == -1    /* indicates that an error occurred  */
      ||numReady ==  0) { /* indicates that a timeout occurred */
      stopPolling = 1;    /* either way, we stop polling */
    }
    else /* positive number -> events occurred, now check which events occurred */
    {
      /* check if data is ready to be received from the rs-485 port */
      if (fds[0].revents & pollevent) {
        /* data is ready to read from the serialport */

        readBytes = read(fd, &data, 1); /* try to read 1 byte from rs485 */

        if (readBytes > 0)
        {
          readCount += readBytes;
          printf("rs485 data: 0x%2X ", data);
          if (data >= 32 && data <= 126)
            printf(" - '%C'", data);
          printf("\n");
        }
        
        /* quit after reading the specified number of bytes */
        if (readCount >= maxReadCount)
        {
          printf("%i bytes received, terminating...\n", maxReadCount);
          stopPolling = 1;
        }
      } 

      /* check if data is ready to be received from the stdin */
      if (fds[1].revents & pollevent) {
        readBytes = read(STDIN_FILENO, &data, 1); /* try to read 1 byte from stdin */

        printf("stdin data: 0x%2X ", data);
          if (data >= 32 && data <= 126)
            printf(" - '%C'", data);
          printf("\n");

        /* quit the loop when the 'q' key was pressed */
        if (data == 'q') {
          printf("q key pressed, terminating...\n");
          stopPolling = 1;
        }
      }

    }

  } while(!stopPolling);

  restoreTermios();
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

/* Initialize new terminal i/o settings */
static void initTermios(int echo) 
{
  tcgetattr(0, &_old); /* grab old terminal i/o settings */
  _new = _old; /* make new settings same as old settings */
  _new.c_lflag &= ~ICANON; /* disable buffered i/o */
  _new.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &_new); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
static void restoreTermios(void) 
{
  tcsetattr(0, TCSANOW, &_old);
}
