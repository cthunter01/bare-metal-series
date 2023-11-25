// C library headers
#include <stdio.h>
#include <string.h>
#include <stdint.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <signal.h>

#define PACKET_LENGTH_BYTES (1)
#define PACKET_DATA_BYTES   (16)
#define PACKET_CRC_BYTES    (1)
#define PACKET_CRC_INDEX    (PACKET_LENGTH_BYTES + PACKET_DATA_BYTES)
#define PACKET_LENGTH       (PACKET_LENGTH_BYTES + PACKET_DATA_BYTES + PACKET_CRC_BYTES)

#define PACKET_ACK_DATA0 (0x15)
#define PACKET_RETX_DATA0 (0x19)

#define PACKET_BUFFER_SIZE (8)

#define BAUD_RATE(RATE) B##RATE

static bool break_loop = false;
static void sig_handler(int sig)
{
    if(sig == SIGINT)
    {
        break_loop = true;
    }
}

struct comms_packet_t
{
    uint8_t len;
    uint8_t data[PACKET_DATA_BYTES];
    uint8_t crc;
};

enum comms_state_t
{
    CommsState_Length,
    CommsState_Data,
    CommsState_CRC,
};

static comms_state_t state = comms_state_t::CommsState_Length;
static uint8_t data_byte_count = 0;

static comms_packet_t temporary_packet = 
{
    .len = 1,
    .data = {0xaa},
    .crc = 0
};
static comms_packet_t retx_packet;
comms_packet_t ack_packet;
comms_packet_t last_xmit_packet = 
{
    .len = 1,
    .data = {0xaa},
    .crc = 0
};

static comms_packet_t packet_buffer[PACKET_BUFFER_SIZE];
static uint32_t packet_read_index = 0;
static uint32_t packet_write_index = 0;
static uint32_t packet_buffer_mask = PACKET_BUFFER_SIZE;

uint8_t crc8(uint8_t* data, uint32_t length);
void comms_packet_copy(const comms_packet_t* src, comms_packet_t* dest);
bool comms_is_retx_packet(const comms_packet_t* packet);
bool comms_is_ack_packet(const comms_packet_t* packet);
bool comms_is_packet_valid(const comms_packet_t* packet);


// returns handle to serial port
int setup_serial_port(const char* path);

int main(int argc, char* argv[])
{
  signal(SIGINT, sig_handler);
  break_loop = false;
  temporary_packet.len = 1;
  for(uint8_t i = 0; i < PACKET_DATA_BYTES; ++i)
  {
    temporary_packet.data[i] = 0xaa;
  }
  temporary_packet.crc = crc8((uint8_t*)(&temporary_packet), PACKET_LENGTH_BYTES + PACKET_DATA_BYTES);
  comms_packet_copy(&temporary_packet, &retx_packet);
  retx_packet.len = 1;
  retx_packet.data[0] = PACKET_RETX_DATA0;
  retx_packet.crc = crc8((uint8_t*)(&retx_packet), PACKET_LENGTH_BYTES + PACKET_DATA_BYTES);
  comms_packet_copy(&temporary_packet, &ack_packet);
  ack_packet.len = 1;
  ack_packet.data[0] = PACKET_ACK_DATA0;
  ack_packet.crc = crc8((uint8_t*)(&ack_packet), PACKET_LENGTH_BYTES + PACKET_DATA_BYTES);
  int serial_port;
  if(argc == 2)
  {
    serial_port = setup_serial_port(argv[1]);
  }
  else
  {
    serial_port = setup_serial_port("/dev/ttyACM2");
  }


  // Allocate memory for read and write buffers
  uint8_t packet_send_buf[sizeof(comms_packet_t)];
  uint8_t packet_recv_buf[sizeof(comms_packet_t)];

int num_bytes_written = 0;
uint32_t read_count = 0;
///////////////////////////////////////////////////////////////////
// BEGIN TESTS
///////////////////////////////////////////////////////////////////

    // TESTS
    // 1. Send regular packet. Expect ack in response
    // 2. Send corrupted packet. Expect retx in response
    // 3. Receive a regular packet
    // 4. Receive a corrupted packet
  // FIRST TEST
  // 
  // Read response
  // Write retx_packet to serial port
  /*
  comms_packet_copy(&retx_packet, (comms_packet_t*)packet_send_buf);
  num_bytes_written = write(serial_port, packet_send_buf, sizeof(packet_send_buf));
  for(unsigned int i = 0; i < num_bytes_written; ++i)
  {
    printf("0x%02x ", packet_send_buf[i]);
  }
  printf("\n");
  sleep(1);

  while(!break_loop)
  {
    read_count = 0;
    while(read_count < sizeof(packet_recv_buf))
    {
        int num_bytes = read(serial_port, packet_recv_buf + read_count, sizeof(packet_recv_buf) - read_count);
        if (num_bytes < 0) {
            printf("Error reading: %s", strerror(errno));
            return 1;
        }
        read_count += num_bytes;
    }

    printf("Read %i bytes.\n", read_count);
    for(unsigned int i = 0; i < read_count; ++i)
    {
        printf("0x%02x ", packet_recv_buf[i]);
    }
    printf("\n");
    printf("Is packet valid?: %s\n", comms_is_packet_valid((comms_packet_t*)(packet_recv_buf)) ? "true" : "false");
    printf("Is packet ack?: %s\n", comms_is_ack_packet((comms_packet_t*)(packet_recv_buf)) ? "true" : "false");
    printf("Is packet retx?: %s\n", comms_is_retx_packet((comms_packet_t*)(packet_recv_buf)) ? "true" : "false");
  }
  */
  
  // Try sending a regular normal packet and get an ack back
  comms_packet_t packet1;
  packet1.len = 9;
  for(unsigned i = 0; i < PACKET_DATA_BYTES; ++i)
  {
    packet1.data[i] = 0xaa;
    if(i < 9)
    {
        packet1.data[i] = i + 1;
    }
  }
//  packet1.data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa };
  packet1.crc = crc8((uint8_t*)(&packet1), PACKET_LENGTH_BYTES + PACKET_DATA_BYTES);

  uint8_t send_buf[PACKET_LENGTH];
  comms_packet_copy(&packet1, (comms_packet_t*)send_buf);

  num_bytes_written = write(serial_port, send_buf, sizeof(packet1));

  for(unsigned int i = 0; i < num_bytes_written; ++i)
  {
    printf("0x%02x ", send_buf[i]);
  }
  printf("\n");
  sleep(1);
read_count = 0;
while(read_count < sizeof(packet_recv_buf))
{
    int num_bytes = read(serial_port, packet_recv_buf + read_count, sizeof(packet_recv_buf) - read_count);
    if (num_bytes < 0) {
        printf("Error reading: %s", strerror(errno));
        return 1;
    }
    read_count += num_bytes;

}
    printf("Read %i bytes.\n", read_count);
    for(unsigned int i = 0; i < read_count; ++i)
    {
        printf("0x%02x ", packet_recv_buf[i]);
    }
    printf("\n");
    printf("Is packet valid?: %s\n", comms_is_packet_valid((comms_packet_t*)(packet_recv_buf)) ? "true" : "false");
    printf("Is packet ack?: %s\n", comms_is_ack_packet((comms_packet_t*)(packet_recv_buf)) ? "true" : "false");
    printf("Is packet retx?: %s\n", comms_is_retx_packet((comms_packet_t*)(packet_recv_buf)) ? "true" : "false");

////////////
// Try sending a bad packet. Do we get a retx back?
  packet1.len = 9;
  for(unsigned i = 0; i < PACKET_DATA_BYTES; ++i)
  {
    packet1.data[i] = 0xaa;
    if(i < 9)
    {
        packet1.data[i] = i + 1;
    }
  }
//  packet1.data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa };
  packet1.crc = crc8((uint8_t*)(&packet1), PACKET_LENGTH_BYTES + PACKET_DATA_BYTES);
  // make it bad
  packet1.crc += 1U;

  send_buf[PACKET_LENGTH];
  comms_packet_copy(&packet1, (comms_packet_t*)send_buf);

  num_bytes_written = write(serial_port, send_buf, sizeof(packet1));

  for(unsigned int i = 0; i < num_bytes_written; ++i)
  {
    printf("0x%02x ", send_buf[i]);
  }
  printf("\n");
  sleep(1);
read_count = 0;
while(read_count < sizeof(packet_recv_buf))
{
    int num_bytes = read(serial_port, packet_recv_buf + read_count, sizeof(packet_recv_buf) - read_count);
    if (num_bytes < 0) {
        printf("Error reading: %s", strerror(errno));
        return 1;
    }
    read_count += num_bytes;

}
    printf("Read %i bytes.\n", read_count);
    for(unsigned int i = 0; i < read_count; ++i)
    {
        printf("0x%02x ", packet_recv_buf[i]);
    }
    printf("\n");
    printf("Is packet valid?: %s\n", comms_is_packet_valid((comms_packet_t*)(packet_recv_buf)) ? "true" : "false");
    printf("Is packet ack?: %s\n", comms_is_ack_packet((comms_packet_t*)(packet_recv_buf)) ? "true" : "false");
    printf("Is packet retx?: %s\n", comms_is_retx_packet((comms_packet_t*)(packet_recv_buf)) ? "true" : "false");

  close(serial_port);
  return 0; // success
};

// implementation taken from Low Byte Productions
// youtube series on STM32 Bare Metal Programming
uint8_t crc8(uint8_t* data, uint32_t length)
{
    uint8_t crc = 0;

    for(uint32_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for(uint8_t j = 0; j < 8; ++j)
        {
            if(crc & 0x80)
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

int setup_serial_port(const char* path)
{
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int serial_port = open(path, O_RDWR);

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 100;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 1;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, BAUD_RATE(115200));
  cfsetospeed(&tty, BAUD_RATE(115200));

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  return serial_port;
}

bool comms_is_retx_packet(const comms_packet_t* packet)
{
    if(packet->len != 1)
    {
        return false;
    }
    if(packet->data[0] != PACKET_RETX_DATA0)
    {
        return false;
    }
    for(uint8_t i = 1; i < PACKET_DATA_BYTES; ++i)
    {
        if(packet->data[i] != 0xaa)
        {
            return false;
        }
    uint8_t crc = crc8((uint8_t*)(packet), PACKET_LENGTH_BYTES + PACKET_DATA_BYTES);
    if(packet->crc != crc)
    {
        return false;
    }
    }
    return true;
}

bool comms_is_ack_packet(const comms_packet_t* packet)
{
    if(packet->len != 1)
    {
        return false;
    }
    if(packet->data[0] != PACKET_ACK_DATA0)
    {
        return false;
    }
    for(uint8_t i = 1; i < PACKET_DATA_BYTES; ++i)
    {
        if(packet->data[i] != 0xaa)
        {
            return false;
        }
    }
    uint8_t crc = crc8((uint8_t*)(packet), PACKET_LENGTH_BYTES + PACKET_DATA_BYTES);
    if(packet->crc != crc)
    {
        return false;
    }
    return true;
}

bool comms_is_packet_valid(const comms_packet_t* packet)
{
    uint8_t crc = crc8((uint8_t*)(packet), PACKET_LENGTH_BYTES + PACKET_DATA_BYTES);
    if(packet->crc != crc)
    {
        return false;
    }
    return true;
}

void comms_packet_copy(const comms_packet_t* src, comms_packet_t* dest)
{
    dest->len = src->len;
    for(uint8_t i = 0; i < PACKET_DATA_BYTES; ++i)
    {
        dest->data[i] = src->data[i];
    }
    dest->crc = src->crc;
}
