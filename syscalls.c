/******************************************************************************/
/*                                                                            */
/*        Syscall support functions for newlib console I/O with stdio         */
/*                                                                            */
/******************************************************************************/

/* $Id: syscalls.c,v 1.4 2008/08/25 16:11:40 cvs Exp $ */
/* 20090508 Nemui Fixed scanf consideration */
/* 20091220 Nemui Separated Device Independent Definitions */
/* 20101107 Nemui Added error handling in sbrk 
            Nemui Adopted to C99              */
/* 20120615 Nemui Fixed _heap_end definitions */
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <reent.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

/* platform dependent definitions */
#include "syscalls_if.h"
#include "stm32f4xx.h"
#include "usbd_cdc_vcp.h"
#include "stm32f4driver.h"

/**************************************************************************/
/* Send 1 character */
inline void putch(uint8_t data)
{
// #if (UART_HANDLING == UART_INTERRUPT_MODE)
//  /* Interrupt Version */
//  while(!USART_TXBuffer_FreeSpace(pUSART_Buf));
//  USART_TXBuffer_PutByte(pUSART_Buf,data);
// #else 
//  /* Polling version */
//  while (!(UART->SR & USART_FLAG_TXE));
//  UART->DR = data;
// #endif
  VCP_put_char(data);
}

/**************************************************************************/
/*! 
    High Level function.
*/
/**************************************************************************/
/* Receive 1 character */
uint8_t getch()
{
// #if (UART_HANDLING == UART_INTERRUPT_MODE)
//  if (USART_RXBufferData_Available(pUSART_Buf))  return USART_RXBuffer_GetByte(pUSART_Buf);
//  else                       return false;
// #else
//  /* Polling version */
//  while (!(UART->SR & USART_FLAG_RXNE));
//  return (uint8_t)(USART->DR);
// #endif
  uint8_t buf;
  VCP_get_char(&buf);
  return buf;
}

/* new code for _read_r provided by Alexey Shusharin - Thanks */
_ssize_t _read_r(struct _reent *r, int file, void *ptr, size_t len)
{
  char c;
  int  i;
  unsigned char *p;

  p = (unsigned char*)ptr;
  for (i = 0; i < len; i++)
  {
	/* 20090521Nemui */
	do{		
		c = getch();
	}while(c == false);
	/* 20090521Nemui */
	
    *p++ = c;
	#ifdef ECHOBACK 
		putch(c);
	#endif
	
    if (c == '\r' && i <= (len - 2)) /* 0x0D */
    {
      *p = '\n';					 /* 0x0A */
	  #ifdef ECHOBACK 
		putch('\n');				 /* 0x0A */
	  #endif
      return i + 2;
    }
  }
  return i;
}

_ssize_t _write_r (
    struct _reent *r, 
    int file, 
    const void *ptr, 
    size_t len)
{
	int i;
	const unsigned char *p;
	
	p = (const unsigned char*) ptr;
	
	for (i = 0; i < len; i++) {
		if (*p == '\n' ) putch('\r');
		putch(*p++);
		Delay(4);
	}
	
	return len;
}

int _close_r(
    struct _reent *r, 
    int file)
{
	return 0;
}

_off_t _lseek_r(
    struct _reent *r, 
    int file, 
    _off_t ptr, 
    int dir)
{
	return (_off_t)0;	/*  Always indicate we are at file beginning.	*/
}


int _fstat_r(
    struct _reent *r, 
    int file, 
    struct stat *st)
{
	/*  Always set as character device.				*/
	st->st_mode = S_IFCHR;	
		/* assigned to strong type with implicit 	*/
		/* signed/unsigned conversion.  Required by 	*/
		/* newlib.					*/

	return 0;
}

#ifdef __GNUC__
int isatty(int file); /* avoid warning */
#endif
int isatty(int file)
{
	return 1;
}


void _exit(int n) {
label:  goto label; /* endless loop */
}

int _getpid(int file)
{
	return 1;
}

int _kill(int file)
{
	return 1;
}

/* "malloc clue function" */
/**** Locally used variables. Nemui Added. ****/
extern char end[];              /*  end is set in the linker command 	*/
								/* file and is the end of statically 	*/
								/* allocated data (thus start of heap).	*/
extern char _heap_end[];		/* Bottom of HEAP region 				*/
static char *heap_end;
static char *heap_ptr=NULL;		/* Points to current end of the heap.	*/

/************************** _sbrk_r *************************************/
/*  Support function.  Adjusts end of heap to provide more memory to	*/
/* memory allocator. Simple and dumb with no sanity checks.				*/
/*  struct _reent *r	-- re-entrancy structure, used by newlib to 	*/
/*			support multiple threads of operation.						*/
/*  ptrdiff_t nbytes	-- number of bytes to add.						*/
/*  Returns pointer to start of new heap area.							*/
/*  Note:  This implementation is not thread safe (despite taking a		*/
/* _reent structure as a parameter).  									*/
/*  Since _s_r is not used in the current implementation, the following	*/
/* messages must be suppressed.	*/

/* Register name faking - works in collusion with the linker.  */
/* Nemui Added */
#ifdef USE_SP_AS_HEAP_END
 #warning "use stackpointer as sbrk's heapend!"
 register char * stack_ptr asm ("sp");
 #define _heap_end stack_ptr
#endif

void * _sbrk_r(
    struct _reent *_s_r, 
    ptrdiff_t nbytes)
{
	char  *base;		/*  errno should be set to  ENOMEM on error	*/

	if (!heap_ptr) {	/*  Initialize if first time through.		*/
		heap_ptr = end;
	}
	base = heap_ptr;	/*  Point to end of heap.					*/
	
	if (heap_ptr + nbytes > _heap_end)
	{
			errno = ENOMEM;
			return (caddr_t) -1;
	}
	heap_ptr += nbytes;	/*  Increase heap.							*/
	
	return base;		/*  Return pointer to start of new heap area.	*/
}



void * _sbrk(ptrdiff_t incr)
{
  char  *base;

/* Initialize if first time through. */

  if (!heap_ptr) heap_ptr = end;

  base = heap_ptr;      /*  Point to end of heap.                       */

	if (heap_ptr + incr > _heap_end)
	{
			errno = ENOMEM;
			return (caddr_t) -1;
	}
  
  heap_ptr += incr;     /*  Increase heap.                              */

  return base;          /*  Return pointer to start of new heap area.   */
}

int _open(const char *path, int flags, ...)
{
  return 1;
}

int _close(int fd)
{
  return 0;
}

int _fstat(int file, struct stat *st)
{
  file = file; /* avoid warning */
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
  file = file; /* avoid warning */
  return 1;
}


int _lseek(int file, int ptr, int dir) {
	file = file; /* avoid warning */
	ptr = ptr; /* avoid warning */
	dir = dir; /* avoid warning */
	return 0;
}

int _read(int fd, char *buf, size_t cnt)
{
  *buf = getch();

  return 1;
}

int _write(int fd, const char *buf, size_t cnt)
{
  int i;

  for (i = 0; i < cnt; i++)
    putch(buf[i]);

  return cnt;
}

/* .ARM.exidx is sorted, so has to go in its own output section.  */
/* Nemui Changed */
extern char *__exidx_start;
extern char *__exidx_end;

/* Override fgets() in newlib with a version that does line editing */
/*
char *fgets(char *s, int bufsize, void *f)
{
  cgets(s, bufsize);
  return s;
}
*/

char* get_heap_end(void)
{
	return (char*) heap_end;
}

char* get_stack_top(void)
{
	return (char*) __get_MSP();
	//return (char*) __get_PSP();
}
