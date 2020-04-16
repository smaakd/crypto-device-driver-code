#include<crypter.h>
#include <sys/mman.h>
#define DEV_LOCATION "/dev/crypto_device"
#define SET_KEY 0
#define SET_CONFIG 1
#define ENCRYPT 2
#define DECRYPT 3
#define MMIO_NO_INT 0
#define MMIO_INT 1
#define DMA_NO_INT 2
#define DMA_INT 3

#define MB 1048576

struct write_to_op{
	int action;
	int cdev;
	uint8_t a;
	uint8_t b;
	config_t type;
	uint8_t value;
	void* addr;
	uint64_t length;
	uint8_t isMapped;
};

struct key_pair
{
  uint8_t a;
  uint8_t b;

  /* data */
};

uint64_t mmap_size;


struct key_pair keys[100];

/*Function template to create handle for the CryptoCard device.
On success it returns the device handle as an integer*/
DEV_HANDLE create_handle()
{
  int fd = open(DEV_LOCATION,O_RDWR);
  // printf("fd = %d\n",fd);
   if(fd < 0){
     
       printf("create_handle: error\n");
       return ERROR;
   }
    return fd;
}

/*Function template to close device handle.
Takes an already opened device handle as an arguments*/
void close_handle(DEV_HANDLE cdev)
{
  close(cdev);
}

/*Function template to encrypt a message using MMIO/DMA/Memory-mapped.
Takes four arguments
  cdev: opened device handle
  addr: data address on which encryption has to be performed
  length: size of data to be encrypt
  isMapped: TRUE if addr is memory-mapped address otherwise FALSE
*/
int encrypt(DEV_HANDLE cdev, ADDR_PTR addr, uint64_t length, uint8_t isMapped)
{
  // printf("encrypt library: %s\n",(char *) addr);
  // printf("buff address = %x\n",addr);
  struct write_to_op* handle = (struct write_to_op* ) malloc(sizeof(struct write_to_op));
  // return ERROR;
  handle->a = keys[cdev].a;
  handle->b = keys[cdev].b;
  // printf("keyb = %d\n",handle->b);ti
  handle->action = ENCRYPT;
  handle->addr = addr;
  handle->length = length;
  handle->isMapped = isMapped;
  // printf("size of jhandle = %d\n",sizeof(struct write_to_op));
  return read(cdev,(void*) handle, sizeof(struct write_to_op));
  // write(cdev,(const void*) addr, 8);
  // char* adr = (char*) addr; 
  // adr[8] = '\0';
  //
}

/*Function template to decrypt a message using MMIO/DMA/Memory-mapped.
Takes four arguments
  cdev: opened device handle
  addr: data address on which decryption has to be performed
  length: size of data to be decrypt
  isMapped: TRUE if addr is memory-mapped address otherwise FALSE
*/
int decrypt(DEV_HANDLE cdev, ADDR_PTR addr, uint64_t length, uint8_t isMapped)
{
  struct write_to_op* handle = (struct write_to_op* ) malloc(sizeof(struct write_to_op));
  // return ERROR;
  handle->a = keys[cdev].a;
  handle->b = keys[cdev].b;
  handle->action = DECRYPT;
  handle->addr = addr;
  handle->length = length;
  handle->isMapped = isMapped;
  // printf("size of jhandle = %d\n",sizeof(struct write_to_op));
  return read(cdev,(void*) handle, sizeof(struct write_to_op));
  // return ERROR;
}

/*Function template to set the key pair.
Takes three arguments
  cdev: opened device handle
  a: value of key component a
  b: value of key component b
Return 0 in case of key is set successfully*/
int set_key(DEV_HANDLE cdev, KEY_COMP a, KEY_COMP b)
{
    keys[cdev].a = a;
    keys[cdev].b = b;
    return 0;
  // struct write_to_op* handle = (struct write_to_op* ) malloc(sizeof(struct write_to_op));
  // handle->action = SET_KEY;
  // handle->a = a;
  // handle->b = b;
  // return read(cdev,(void*) handle, 10);
  // return ERROR;
}

/*Function template to set configuration of the device to operate.
Takes three arguments
  cdev: opened device handle
  type: type of configuration, i.e. set/unset DMA operation, interrupt
  value: SET/UNSET to enable or disable configuration as described in type
Return 0 in case of key is set successfully*/
int set_config(DEV_HANDLE cdev, config_t type, uint8_t value)
{
  struct write_to_op* handle = (struct write_to_op* ) malloc(sizeof(struct write_to_op));
  // if((type == SET) && (value == SET))
  
  handle->action = SET_CONFIG;
  handle->type = type;
  handle->value = value;
  return read(cdev,(void*) handle, 10);
}

/*Function template to device input/output memory into user space.
Takes three arguments
  cdev: opened device handle
  size: amount of memory-mapped into user-space (not more than 1MB strict check)
Return virtual address of the mapped memory*/
ADDR_PTR map_card(DEV_HANDLE cdev, uint64_t size)
{
  void* addr = mmap(NULL,MB, PROT_READ| PROT_WRITE, MAP_PRIVATE ,cdev, 0);
  // printf("size of handle = %lx\n",addr);
  mmap_size = size;
  // int res = munmap(addr,size);
  // printf("map_card: buf = %s\n", (char *) addr);                
  return addr + 0xa8;
}

/*Function template to device input/output memory into user space.
Takes three arguments
  cdev: opened device handle
  addr: memory-mapped address to unmap from user-space*/
void unmap_card(DEV_HANDLE cdev, ADDR_PTR addr)
{
    munmap(addr,mmap_size);
    mmap_size = 0;
}
