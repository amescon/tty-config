/* test tool for driver configuration */
#include <stdio.h>
//#include <iostream>
#include <unistd.h> /* needed for close() */
#include <termios.h> // needed for implementing set_inferface_attribs()
#include <fcntl.h>   /* needed for O_RDWR, O_NOCTTY, O_NDELAY */
#include <string.h>
#include <getopt.h>  /* for getopt_long*/
#include <stdlib.h>  /* for atoi() */

/* function to configure the serial port */
int set_interface_attribs (int fd, int speed, int parity, int stopbits, int databits)
{
  struct termios tty = {};
  //memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
  {
    //error_message ("error %d from tcgetattr", errno);
    return -1;
  }

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  //tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  tty.c_cflag = (tty.c_cflag & ~CSIZE);
  tty.c_cflag |= databits;

  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  //tty.c_cflag |= CS8;
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
  tty.c_cflag |= stopbits;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
  {
    //error_message ("error %d from tcsetattr", errno);
    return -1;
  }
  return 0;
}

/* forward declarations */
static void ttyc_printMsg(char* msg, int count);
static void ttyc_printMsgHex(char* msg);
static void ttyc_handle_parameters(int argc, char** argv);
static int ttyc_open_device(void);
static void ttyc_apply_config(void);

/* variables */
char DEVICE[255] = { "" };
char BUFFER[255]; /* read buffer */
int FD = -1;

/* device configuration configuration */
int BAUD = B9600;   /* defaults to baudrate 9600 */
int PARITY = 0;     /* defaults to no parity */
int STOPBITS = 0;   /* defaults to no stopbits */
int DATABITS = CS8; /* defaults to 8 databits */

static int ttyc_open_device()
{
  int fd;

  /* close old device */
  if (FD != -1) {
    printf("closing device: %i\n", FD);
    close (FD);
  }

  /* opening device */
  printf("opening device %s\n", DEVICE );
  fd = open(DEVICE, O_RDWR | O_NOCTTY  | O_NDELAY);
  if (fd < 0) {
    printf("open failed!\n");
  }
  else
    FD = fd;

  return fd;
}

static void ttyc_apply_config()
{
  set_interface_attribs(FD, BAUD, PARITY, STOPBITS, DATABITS);
}

static void ttyc_print_usage()
{
  printf("tty config tool by mdk\n");
  printf("example usage: ./tty-config --device=/dev/ttyRPC+0 --baudrate=9600 --databits=8 --parity=none --stopbits=1\n");
}

/* main entry point */
int main(int argc, char *argv[])
{
  int fd, bytesRead;
  int i;
  
  ttyc_print_usage();
  
  /* handle the params */
  ttyc_handle_parameters(argc, argv);

  /* open the device */
  if (ttyc_open_device() != -1)
  {

    /* apply the configuration */
    ttyc_apply_config();

    /* closing device */
    printf("closing device %i\n", fd);
    close(fd);
  }
}

typedef enum
{
  ttyc_command_set_device,
  ttyc_command_set_baudrate,
  ttyc_command_set_parity,
  ttyc_command_set_stopbits,
  ttyc_command_set_databits,
  ttyc_command_wait
} test_command_t;

static void ttyc_set_baudrate(char* arg)
{
  int i;
  printf("set baudrate: %s\n", arg);

  i = atoi(arg);
  switch (i)
  {
    case 9600: BAUD   = B9600; break;
    case 19200: BAUD  = B19200; break;
    case 38400: BAUD  = B38400; break;
    case 57600: BAUD  = B57600; break;
    case 115200: BAUD = B115200; break;
    default: printf("unknown baudrate, try 9600, 19200, 38400, 57600 or 115200\n"); break;
  }
}

static void ttyc_set_parity(char* arg)
{
  printf("set parity: %s\n", arg);
  if (strncmp(arg, "none", 4)==0)
    PARITY = 0;
  else if (strncmp(arg, "even", 4) == 0)
    PARITY = PARENB;
  else if (strncmp(arg, "odd", 3) == 0)
    PARITY = (PARENB | PARODD);
  else
    printf("unknown parity value, allowed values are: none, even, odd\n");
}

static void ttyc_set_stopbits(char* arg)
{
  int i;
  printf("set stopbits: %s\n", arg);

  i = atoi(arg);

  if (i == 1)
    STOPBITS = 0;
  else if (i == 2)
    STOPBITS = CSTOPB;
  else
    printf("valid values: 0, 1\n");
}

static void ttyc_set_databits(char* arg)
{
  int i;
  printf("set databits: %s\n", arg);
  i = atoi(arg);

  switch (i)
  {
    case 5: DATABITS = CS5; break;
    case 6: DATABITS = CS6; break;
    case 7: DATABITS = CS7; break;
    case 8: DATABITS = CS8; break;
    default: printf("valid values: 5, 6, 7, 8\n"); break;
  }
}

static void ttyc_wait(char* arg)
{
  int i, j;

  printf("waiting: %s\n", arg);
  i = atoi(arg);
  for (j = 0; j < i; j++)
    usleep(1000000);
}

static void ttyc_execute_command(test_command_t cmd, char* arg)
{
  switch (cmd)
  {
    case ttyc_command_set_device:
      printf("set_device: %s\n", arg);
      strncpy(DEVICE, arg, 255);
      break;

    case ttyc_command_set_baudrate:
      ttyc_set_baudrate(arg);
      break;

    case ttyc_command_set_parity:
      ttyc_set_parity(arg);
      break;

    case ttyc_command_set_stopbits:
      ttyc_set_stopbits(arg);
      break;

    case ttyc_command_set_databits:
      ttyc_set_databits(arg);
      break;

    case ttyc_command_wait:
      ttyc_wait(arg);
      break;
  }
}

static void ttyc_handle_parameters(int argc, char** argv)
{
  int c = -1;

  struct option long_options[] = {
    { "device",     required_argument, 0, 0 },
    { "baudrate",   required_argument, 0, 0 },
    { "parity",     required_argument, 0, 0 },
    { "stopbits",   required_argument, 0, 0 },
    { "databits",   required_argument, 0, 0 },
    { "wait",       required_argument, 0, 0 },
    { 0,            0,                 0, 0 }
  };

  /* enable error messages for arguments */
  opterr = 1;

  /* getopt_long stores the option index here */
  int option_index = 0;

  do
  {
    c = getopt_long(argc, argv, "", long_options, &option_index);

    if (c != -1)
      ttyc_execute_command((test_command_t)option_index, optarg);

  } while (c != -1);

}

static void ttyc_printMsg(char* msg, int count)
{
  int i;
  printf("(%i) [", count);
  for (i = 0; i < count; i++)
  {
    printf( (i + 1 == count) ? "0x%02X" : "0x%02X ", msg[i]);
  }
  printf("]\n");
}

static void ttyc_printMsgHex(char* msg)
{
  int i, length = 0;
  while (msg[length++] != 0);
  length--;

  printf("[");
  for (i = 0; i < length; i++)
  {
    switch (msg[i])
    {
      case '\r': printf("  \\r"); break;
      case '\n': printf("  \\n"); break;
      default:
        printf ("%4c", msg[i]);
        break;
    }
    if (i + 1 < length)
      printf(" ");
  }
  printf("]");

  printf("\n");
  
  printf("[");
  for (i = 0; i < length; i++)
  {
    printf ("0x%02X", msg[i]);
    if (i + 1 < length)
      printf(" ");
  }
  printf("]");

  printf("\n");
}
