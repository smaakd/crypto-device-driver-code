#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <crypter.h>

// int main()
// {
//   DEV_HANDLE cdev;
//   char msg[128];
//   sprintf(msg,"Hello CS");
//   uint64_t size = strlen(msg);
//   KEY_COMP a=30, b=17;
//   cdev = create_handle();

//   if(cdev == ERROR)
//   {
//     printf("Unable to create handle for device\n");
//     exit(0);
//   }

//   if(set_key(cdev, a, b) == ERROR){
//     printf("Unable to set key\n");
//     exit(0);
//   }
//   encrypt(cdev, msg, size, 0);
//   printf("Encrypted Text: %s\n", msg);
//   close_handle(cdev);
//   return 0;
// }

// void copy(char* addr1,char* addr1)

int main()
{
  DEV_HANDLE cdev;
  char *msg = "Hello CS730!";
  char op_text[16];
  KEY_COMP a=30, b=17;
  uint64_t size = strlen(msg);
  strcpy(op_text, msg);
  cdev = create_handle();

  // Setting the DMA
  set_config(cdev, DMA, UNSET);

  // Setting the Interrupt
  set_config(cdev, INTERRUPT, UNSET);

  if(cdev == ERROR)
  {
    printf("Unable to create handle for device\n");
    exit(0);
  }

  if(set_key(cdev, a, b) == ERROR){
    printf("Unable to set key\n");
    exit(0);
  }

  printf("Original Text: %s\n", msg);

  // encrypt(cdev, op_text, size, 0);
  // printf("Encrypted Text: %s\n", op_text);

  // decrypt(cdev, op_text, size, 0);
  // printf("Decrypted Text: %s\n", op_text);
  char* addr = (char *)map_card(cdev,20);
  // addr[0] = 'S';
  printf("int = %lx\n",(addr));
  *(addr) = '%';
  //  strncpy(op_text,addr,strlen(msg));
  printf("int = %c\n",*((char*)addr));
  //  printf("pre before Encrypted Text: %s\n", addr);
  strncpy(addr,op_text,strlen(msg));
  // printf("before Encrypted Text: %s\n", addr);
  encrypt(cdev, 0, strlen(msg), 1);
  // printf("int = %s\n",*((char*)addr));
  // printf("after Encrypted Text: %s\n", addr);
  strncpy(op_text,addr,2);
  printf("TRansfer success Text:\n");
  printf("retreived Text: %s\n", op_text);
  unmap_card(cdev,addr);

  // printf("------ Original Text: %s\n", msg);
  close_handle(cdev);
  return 0;
}
