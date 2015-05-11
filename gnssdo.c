#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <pwd.h>
#include <errno.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include "gpsdthread.h"

// register address
static const size_t ECAP0_BASE_ADDRESS = 0x48300100;
static const size_t ECAP2_BASE_ADDRESS = 0x48304100;
static const size_t DMTIMER4_BASE_ADDRESS = 0x48044000;
static const size_t DMTIMER5_BASE_ADDRESS = 0x48046000;
static const size_t CM_PER_BASE_ADDRESS = 0x44E00000;
static const size_t CM_DPLL_BASE_ADDRESS = 0x44E00500;
static const size_t EPWM1_BASE_ADDRESS = 0x48302200;

static const size_t DMTIMER_TCLR = 0x38;
static const size_t DMTIMER_TCRR = 0x3C;
static const size_t DMTIMER_TLDR = 0x40;
static const size_t CLKSEL_TIMER4_CLK = 0x10;
static const size_t CLKSEL_TIMER5_CLK = 0x10;
static const size_t CM_PER_EPWMSS0_CLKCTRL = 0xD4;
static const size_t CM_PER_EPWMSS1_CLKCTRL = 0xCC;
static const size_t CM_PER_EPWMSS2_CLKCTRL = 0xD8;
static const size_t CM_PER_TIMER4_CLKCTRL = 0x88;
static const size_t CM_PER_TIMER5_CLKCTRL = 0xEC;
static const size_t ECAP_TSCTR = 0x00;
static const size_t ECAP_CAP1 = 0x08;
static const size_t ECAP_ECCTL1 = 0x28;
static const size_t ECAP_ECCTL2 = 0x2A;
static const size_t ECAP_ECFLG = 0x2E;
static const size_t ECAP_ECCLR = 0x30;
static const size_t EPWM_TBCTL = 0x00;
static const size_t EPWM_TBPRD = 0xA;
static const size_t EPWM_CMPA = 0x12;
static const size_t EPWM_AQCTLA = 0x16;
static const size_t EPWM_HRCNFG = 0xC0;
static const size_t EPWM_CMPAHR = 0x10;

// block size
static const size_t ECAP_MEM_SIZE = 0x0180;
static const size_t DMTIMER_MEM_SIZE = 0x1000;
static const size_t CM_PER_MEM_SIZE = 0x400;
static const size_t CM_DPLL_MEM_SIZE = 0x100;
static const size_t PIN_CONF_MEM_SIZE = 0x300;
static const size_t EPWM_MEM_SIZE = 0x60;

static const uint32_t TIMER_CLOCK_FREQ = 10000000;
static const int32_t ECAP_CLOCK_FREQ = 100000000;
// PWM MEP scale factor
// This is empirically determined, since TI docs (SPRUH73K)
// are clearly wrong.
static const int32_t MEP_SF = 121; 

#define UNPRIV_USER "nobody"

// PID loop constants
static const double I_factor = 1.5e-9;
static const double P_factor = 2e-6;

#define MAX_ALLOWED_PHASE_ERR 100000 // 1 ms
#define INIT_PHASE_ERR_COUNTDOWN 3

void *map_mem_region(size_t baseAddr, size_t memSize, int mem_fd)
{

	long pageSize = sysconf(_SC_PAGE_SIZE);

	// round wanted base address down to page boundary
	size_t pageBaseAddr = baseAddr / (size_t)pageSize;
	pageBaseAddr *= (size_t)pageSize;
	
	// offset of wanted address into page
	size_t addrOffset = baseAddr - pageBaseAddr;

	// map
	void *map_addr = mmap(0, memSize + addrOffset, PROT_READ | PROT_WRITE,
		MAP_SHARED, mem_fd, pageBaseAddr);

	// map was successful
	if (map_addr != MAP_FAILED)
	{
		// return pointer to (mapped) wanted base address
		return (void *)((unsigned char *)map_addr + addrOffset);
	}
        else {
		return NULL;
	}

}

int unmap_mem_region(void **addr_ptr, size_t memSize)
{
  long pageSize = sysconf(_SC_PAGE_SIZE);

  // round address down to page boundary

  size_t pageAddr = (size_t)(*addr_ptr) / (size_t)pageSize;
  pageAddr *= (size_t)pageSize;

  // unmap

  int ret = munmap((void *)pageAddr, memSize);
  if (ret == 0)
    *addr_ptr = NULL; /* invalidate the address */

  return ret;
}

int drop_privs(void)
{
  struct passwd *pws;

  errno = 0;
  pws = getpwnam(UNPRIV_USER);

  if (pws == NULL) {
    if (errno != 0)
      perror("getpwnam");
    else
      fprintf(stderr, "User %s not in /etc/passwd\n", UNPRIV_USER);
    return 1;
  }

  if (setgid(pws->pw_gid)) {
    perror("setegid");
    return 1;
  }

  if (setuid(pws->pw_uid)) {
    perror("seteuid");
    return 1;
  }

  return 0;
}

#define WR_REG32(reg_addr, val) *((uint32_t *)(reg_addr)) = (val)
#define RD_REG32(reg_addr) *((uint32_t *)(reg_addr))
#define WR_REG16(reg_addr, val) *((uint16_t *)(reg_addr)) = (val)
#define RD_REG16(reg_addr) *((uint16_t *)(reg_addr))

int main(void)
{
	struct timespec ts;
	pthread_t gpsdth;
	struct gpsdthread_context gpsdctx = {PTHREAD_MUTEX_INITIALIZER, 0};
	int fix_status = 0, r;

	/* Open physical memory file and map the
	   regions we need to access hardware
	*/

	int fd = open("/dev/mem", O_RDWR);
	if (fd == -1) {
		perror("/dev/mem");
		return 1;
	}

	void *cm_per_addr = map_mem_region(CM_PER_BASE_ADDRESS,
					   CM_PER_MEM_SIZE, fd);
	if (cm_per_addr == NULL) {
		perror("CM_PER_BASE_ADDRESS map failed");
		return 1;
	}

	void *cm_dpll_addr = map_mem_region(CM_DPLL_BASE_ADDRESS,
					    CM_DPLL_MEM_SIZE, fd);
	if (cm_dpll_addr == NULL) {
		perror("CM_DPLL_BASE_ADDRESS map failed");
		return 1;
	}

	void *dmtimer4_addr = map_mem_region(DMTIMER4_BASE_ADDRESS,
					     DMTIMER_MEM_SIZE, fd);
	if (dmtimer4_addr == NULL) {
		perror("DMTIMER4_BASE_ADDRESS map failed");
		return 1;
	}

	void *epwm1_addr = map_mem_region(EPWM1_BASE_ADDRESS,
					  EPWM_MEM_SIZE, fd);
	if (epwm1_addr == NULL) {
		perror("EPWM1_BASE_ADDRESS map failed");
		return 1;
	}

	void *ecap0_addr = map_mem_region(ECAP0_BASE_ADDRESS,
					  ECAP_MEM_SIZE, fd);
	if (ecap0_addr == NULL) {
		perror("ECAP0_BASE_ADDRESS map failed");
		return 1;
	}

	void *ecap2_addr = map_mem_region(ECAP2_BASE_ADDRESS,
					  ECAP_MEM_SIZE, fd);
	if (ecap2_addr == NULL) {
		perror("ECAP2_BASE_ADDRESS map failed");
		return 1;
	}

        /* We don't need to do any more maps now, so close the file */
 
       close(fd);

       /* Try to set realtime scheduling */

       int old_sched_policy = sched_getscheduler(0);
       if (old_sched_policy == -1) {
	 perror("sched_getscheduler");
	 return 1;
       }
       struct sched_param old_sched_param, sp;
       if (sched_getparam(0, &old_sched_param)) {
	   perror("sched_getparam");
	   return 1;
       }

       sp.sched_priority = 10;
       r = sched_setscheduler(0, SCHED_FIFO, &sp);
       if (r != 0)
	 perror("sched_setscheduler");

       /* try dropping privileges */

       if (drop_privs())
	 fprintf(stderr, "Failed to drop privs.\n");
       else
	 fprintf(stderr, "Successfully dropped privs.\n");

       /* Start GPS status thread */

       pthread_attr_t gpsdth_attr;
       r = pthread_attr_init(&gpsdth_attr);
       if (r != 0) {
	 fprintf(stderr, "pthread_attr_init: %s",strerror(r));
	 return 1;
       }
       
       pthread_attr_setschedpolicy(&gpsdth_attr, old_sched_policy);
       pthread_attr_setschedparam(&gpsdth_attr, &old_sched_param);
       pthread_attr_setinheritsched(&gpsdth_attr, PTHREAD_EXPLICIT_SCHED);

       r = pthread_create(&gpsdth, &gpsdth_attr, gpsdthread, (void *)&gpsdctx);
       if (r != 0) {
	 fprintf(stderr, "pthread_create: %s\n", strerror(r));
	 return 1;
       }

	// --- Enable module clocks ---

	void *reg_addr = (char *)cm_per_addr + CM_PER_TIMER4_CLKCTRL;
	printf("%%CM_PER_TIMER4_CLKCTRL = 0x%08X\n", RD_REG32(reg_addr));	
	WR_REG32(reg_addr, 0x00000002);


	reg_addr = (char *)cm_per_addr + CM_PER_EPWMSS0_CLKCTRL;
	printf("%%CM_PER_EPWMSS0_CLKCTRL = 0x%08X\n", RD_REG32(reg_addr));	
	WR_REG32(reg_addr, 0x00000002);

	reg_addr = (char *)cm_per_addr + CM_PER_EPWMSS1_CLKCTRL;
	printf("%%CM_PER_EPWMSS1_CLKCTRL = 0x%08X\n", RD_REG32(reg_addr));	
	WR_REG32(reg_addr, 0x00000002);
	//*((uint32_t *)reg_addr) = 0x00030000; // disable


	reg_addr = (char *)cm_per_addr + CM_PER_EPWMSS2_CLKCTRL;
	printf("%%CM_PER_EPWMSS2_CLKCTRL = 0x%08X\n", RD_REG32(reg_addr));	
	WR_REG32(reg_addr, 0x00000002);

	if (unmap_mem_region(&cm_per_addr, CM_PER_MEM_SIZE))
	     perror("munmap");

	// --- Configure TIMER4 to use TCLKIN ---

	reg_addr = (char *)cm_dpll_addr + CLKSEL_TIMER4_CLK;
	printf("%%CLKSEL_TIMER4_CLK = 0x%08X\n", RD_REG32(reg_addr));	
	WR_REG32(reg_addr, 0x0); // Use TCLKIN as clock

	if (unmap_mem_region(&cm_dpll_addr, CM_DPLL_MEM_SIZE))
	  perror("munmap");

	// --- Configure DMTIMER4 to divide 10 MHz to 1 Hz ---

	reg_addr = (char *)dmtimer4_addr + DMTIMER_TCLR;
	WR_REG32(reg_addr, 0x0); // stop timer

	uint32_t timer_load_count = 0xFFFFFFFF - TIMER_CLOCK_FREQ + 1;

	// write load count into count register and load register

	reg_addr = (char *)dmtimer4_addr + DMTIMER_TCRR;
	WR_REG32(reg_addr, timer_load_count);

	reg_addr = (char *)dmtimer4_addr + DMTIMER_TLDR;
	WR_REG32(reg_addr, timer_load_count);

	WR_REG32((char *)dmtimer4_addr + DMTIMER_TCLR, 0x0403); // enable auto-reload timer

	// read count
	reg_addr = (char *)dmtimer4_addr + 0x3C;
	printf("%%register = 0x%08X\n", RD_REG32(reg_addr));	

	// --- Configure PWM1 to drive TCXO control voltage ---

	reg_addr = (char *)epwm1_addr + EPWM_TBPRD;
	printf("%%ePWM1 TBPRD = 0x%04X\n", RD_REG16(reg_addr));	
	WR_REG16(reg_addr, 0xFFFF); // max period

	reg_addr = (char *)epwm1_addr + EPWM_CMPA;
	printf("%%ePWM1 CMPA = 0x%04X\n", RD_REG16(reg_addr));	
	WR_REG16(reg_addr, 0x8000); // duty cycle

	reg_addr = (char *)epwm1_addr + EPWM_AQCTLA;
	printf("%%ePWM1 AQCTLA = 0x%04X\n", RD_REG16(reg_addr));	
	WR_REG16(reg_addr, 0x0012); // high on zero, low on CMPA

	reg_addr = (char *)epwm1_addr + EPWM_HRCNFG;
	printf("%%ePWM1 HRCNFG = 0x%04X\n", RD_REG16(reg_addr));
	// HRPWM configuration
	// Insert delay in falling edge
	// TI docs (SPRUH73K) are a bit confusing about the "HRLOAD"
	// bit in this register.
	WR_REG16(reg_addr, 0x000A);

	WR_REG16((char *)epwm1_addr + EPWM_CMPAHR, 0x0000); // disable HR for now

	reg_addr = (char *)epwm1_addr + EPWM_TBCTL;
	printf("%%ePWM1 TBCTL = 0x%04X\n", RD_REG16(reg_addr));	
	WR_REG16(reg_addr, 0x0000); // count up

	// --- Configure eCAP0,2 to capture GNSS,TCXO PPS events ---

	reg_addr = (char *)ecap2_addr + ECAP_ECCTL2;
	printf("%%eCAP2 ECCTL2 = 0x%04X\n", RD_REG16(reg_addr));	
	WR_REG16(reg_addr, 0x0030); // start counter, enable SYNCIN

	reg_addr = (char *)ecap0_addr + ECAP_ECCTL1;
	printf("%%eCAP0 ECCTL1 = 0x%04X\n", RD_REG16(reg_addr));	
	WR_REG16(reg_addr, 0x0100); // enable CAP loading

	reg_addr = (char *)ecap2_addr + ECAP_ECCTL1;
	printf("%%eCAP2 ECCTL1 = 0x%04X\n", RD_REG16(reg_addr));	
	WR_REG16(reg_addr, 0x0100); // enable CAP loading

	reg_addr = (char *)ecap0_addr + ECAP_ECCTL2;
	printf("%%eCAP0 ECCTL2 = 0x%04X\n", RD_REG16(reg_addr));	
	WR_REG16(reg_addr, 0x0130); // start counter, SWSYNC

	// read count
	reg_addr = (char *)ecap2_addr + ECAP_TSCTR;
	printf("%%eCAP2 TSCTR = 0x%08X\n", RD_REG32(reg_addr));	

	// read count
	reg_addr = (char *)ecap0_addr + ECAP_TSCTR;
	printf("%%eCAP0 TSCTR = 0x%08X\n", RD_REG32(reg_addr));	

	// clear interrupt flags
	reg_addr = (char *)ecap0_addr + ECAP_ECCLR;
	WR_REG16(reg_addr, 0x00FF);
	reg_addr = (char *)ecap2_addr + ECAP_ECCLR;
	WR_REG16(reg_addr, 0x00FF);

	// --- Wait for GNSS fix to be valid ---

	ts.tv_sec = 1;
	ts.tv_nsec = 0;

	for (;;) {
	  
	  pthread_mutex_lock(&gpsdctx.mutex);

	  fix_status = gpsdctx.status;

	  pthread_mutex_unlock(&gpsdctx.mutex);

	  if (fix_status > 0)
	    break;
	  else
	    fprintf(stderr, "Fix not yet valid\n");
	  
	  // wait a bit
	  nanosleep(&ts, NULL);

	}

	// --- Calibrate VCXO tuning range ---

	printf("%%Calibrating VCXO tuning range...\n");

	ts.tv_sec = 0;
	ts.tv_nsec = 10000000; // 10 ms

	// 75% max. control voltage
	WR_REG16((char *)epwm1_addr + EPWM_CMPA, 0xC000);
	sleep(1); // wait for RC delay	

	// reset capture flags
	WR_REG16((char *)ecap0_addr + ECAP_ECCLR, 0x0002);
	WR_REG16((char *)ecap2_addr + ECAP_ECCLR, 0x0002);

	int32_t gnss_pps_cap, tcxo_pps_cap, prev_gnss_pps_cap, prev_tcxo_pps_cap;
	int gnss_count = 0, tcxo_count = 0;
	double gnss_period = 0, tcxo_period = 0;
	const int cal_pps_count = 5;

	while ((gnss_count <= cal_pps_count) || (tcxo_count <= cal_pps_count)) {
		// test for PPS events
		uint16_t ecap0_ecflg = RD_REG16((char *)ecap0_addr + ECAP_ECFLG);
		uint16_t ecap2_ecflg = RD_REG16((char *)ecap2_addr + ECAP_ECFLG);

		if (ecap0_ecflg & 0x0002) {
			// get captured count and clear flag
			gnss_pps_cap = (int32_t)RD_REG32((char *)ecap0_addr + ECAP_CAP1);
			WR_REG16((char *)ecap0_addr + ECAP_ECCLR, 0x0002);

			if (gnss_count > 0)
				gnss_period += (double)(gnss_pps_cap - prev_gnss_pps_cap);

			prev_gnss_pps_cap = gnss_pps_cap;
			gnss_count++;
		}

		if (ecap2_ecflg & 0x0002) {
			// get captured count and clear flag
			tcxo_pps_cap = (int32_t)RD_REG32((char *)ecap2_addr + ECAP_CAP1);
			WR_REG16((char *)ecap2_addr + ECAP_ECCLR, 0x0002);

			if (tcxo_count > 0)
				tcxo_period += (double)(tcxo_pps_cap - prev_tcxo_pps_cap);

			prev_tcxo_pps_cap = tcxo_pps_cap;
			tcxo_count++;
		}		

		nanosleep(&ts, NULL);

	}

	double tcxo_max_freq = (double)gnss_period / (double)tcxo_period;

	printf("%%TCXO max freq = %.7f\n", tcxo_max_freq);

	// 25% max. control voltage
	WR_REG16((char *)epwm1_addr + EPWM_CMPA, 0x4000);
	sleep(1); // wait for RC delay	

	// reset capture flags
	WR_REG16((char *)ecap0_addr + ECAP_ECCLR, 0x0002);
	WR_REG16((char *)ecap2_addr + ECAP_ECCLR, 0x0002);


	gnss_count = 0;
	tcxo_count = 0;
	gnss_period = 0;
	tcxo_period = 0;

	while ((gnss_count <= cal_pps_count) || (tcxo_count <= cal_pps_count)) {
		// test for PPS events
		uint16_t ecap0_ecflg = RD_REG16((char *)ecap0_addr + ECAP_ECFLG);
		uint16_t ecap2_ecflg = RD_REG16((char *)ecap2_addr + ECAP_ECFLG);

		if (ecap0_ecflg & 0x0002) {
			// get captured count and clear flag
			gnss_pps_cap = (int32_t)RD_REG32((char *)ecap0_addr + ECAP_CAP1);
			WR_REG16((char *)ecap0_addr + ECAP_ECCLR, 0x0002);

			if (gnss_count > 0)
				gnss_period += (double)(gnss_pps_cap - prev_gnss_pps_cap);

			prev_gnss_pps_cap = gnss_pps_cap;
			gnss_count++;
		}

		if (ecap2_ecflg & 0x0002) {
			// get captured count and clear flag
			tcxo_pps_cap = (int32_t)RD_REG32((char *)ecap2_addr + ECAP_CAP1);
			WR_REG16((char *)ecap2_addr + ECAP_ECCLR, 0x0002);

			if (tcxo_count > 0)
				tcxo_period += (double)(tcxo_pps_cap - prev_tcxo_pps_cap);

			prev_tcxo_pps_cap = tcxo_pps_cap;
			tcxo_count++;
		}		

		nanosleep(&ts, NULL);

	}

	double tcxo_min_freq = (double)gnss_period / (double)tcxo_period;

	printf("%%TCXO min freq = %.7f\n", tcxo_min_freq);

	double est_duty_cycle = (0.5 - 0.75 * tcxo_min_freq
				 + 0.25 * tcxo_max_freq)
	  / (tcxo_max_freq - tcxo_min_freq);

	if ((est_duty_cycle < 0) || (est_duty_cycle > 1)) {
		printf("%%Duty cycle for correct tuning is out of range!\n");
		est_duty_cycle = 0.5;
	}

	for (;;) {

	  // --- Start 10 MHz divider in sync with GNSS PPS ---

	// set estimated control voltage for correct tuning
	WR_REG16((char *)epwm1_addr + EPWM_CMPA,
		 (uint16_t)(65536 * est_duty_cycle));

	WR_REG32((char *)dmtimer4_addr + DMTIMER_TCLR, 0x0); // stop timer

	int32_t phase_err = 0;
	double phase_err_int = 0;

	// wait for GNSS PPS and start timer
	
	printf("%%Waiting for GNSS PPS...\n");
	ts.tv_sec = 0;
	ts.tv_nsec = 10000000;
	WR_REG16((char *)ecap0_addr + ECAP_ECCLR, 0x0002); // clear flag
	while (!(RD_REG16((char *)ecap0_addr + ECAP_ECFLG) & 0x0002))
		nanosleep(&ts, NULL);

	gnss_pps_cap = (int32_t)RD_REG32((char *)ecap0_addr + ECAP_CAP1);
	int32_t gnss_pps_cnt = (int32_t)RD_REG32((char *)ecap0_addr + ECAP_TSCTR);
	int32_t cnt_elapsed = gnss_pps_cnt - gnss_pps_cap;
	cnt_elapsed = cnt_elapsed / (ECAP_CLOCK_FREQ / TIMER_CLOCK_FREQ);

	// correct start time of counter
	WR_REG32((char *)dmtimer4_addr + DMTIMER_TCRR, timer_load_count
		 + cnt_elapsed);	
        
	reg_addr = (char *)dmtimer4_addr + DMTIMER_TCLR;
        WR_REG32(reg_addr, 0x0403); // enable auto-reload timer

        printf("%%Elapsed since GNSS PPS = %d (DMTIMER clocks)\n", cnt_elapsed);

	if (cnt_elapsed > (TIMER_CLOCK_FREQ - 1))
	  printf("%% WARNING: elapsed count too large!\n");

	printf("%%Timer started\n");

	// --- Main control loop ---

	uint16_t pwm_ctrl_hr = 0;
	uint16_t pwm_ctrl = (uint16_t)(65536 * est_duty_cycle);

	int tcxo_pps_detected = 0;
	int phase_err_countdown = INIT_PHASE_ERR_COUNTDOWN;

	phase_err_int = 2 * (est_duty_cycle - 0.5) / I_factor;

	// clear capture flags
	WR_REG16((char *)ecap0_addr + ECAP_ECCLR, 0x0002);
	WR_REG16((char *)ecap2_addr + ECAP_ECCLR, 0x0002);

	for (;;) {
	  // test for PPS events
	  uint16_t ecap0_ecflg = RD_REG16((char *)ecap0_addr + ECAP_ECFLG);
	  uint16_t ecap2_ecflg = RD_REG16((char *)ecap2_addr + ECAP_ECFLG);

	  if (ecap2_ecflg & 0x0002) {

	    // TCXO PPS event
	    // Store timestamp and reset eCAP
	    tcxo_pps_cap = (int32_t)RD_REG32((char *)ecap2_addr
					     + ECAP_CAP1);
	    WR_REG16((char *)ecap2_addr + ECAP_ECCLR, 0x0002);
	    tcxo_pps_detected = 1;
	  }

	  if (ecap0_ecflg & 0x0002) {

	    // GNSS PPS event
	    gnss_pps_cap = (int32_t)RD_REG32((char *)ecap0_addr
					     + ECAP_CAP1);
	    WR_REG16((char *)ecap0_addr + ECAP_ECCLR, 0x0002);

	    if (tcxo_pps_detected)
	      phase_err = tcxo_pps_cap - gnss_pps_cap;
	    else
	      phase_err = 0;

	    if (phase_err < -(ECAP_CLOCK_FREQ / 2))
	      phase_err += ECAP_CLOCK_FREQ;
	    
	    if (phase_err > (ECAP_CLOCK_FREQ / 2))
	      phase_err -= ECAP_CLOCK_FREQ;

	    /* Check for valid fix */

	    pthread_mutex_lock(&gpsdctx.mutex);
	    fix_status = gpsdctx.status;
	    pthread_mutex_unlock(&gpsdctx.mutex);

	    if (fix_status > 0) {

	      // check for excessive phase error

	      if (abs(phase_err) > MAX_ALLOWED_PHASE_ERR) {

		printf("%% Phase error exceeds limit. Countdown = %d\n",
		       phase_err_countdown);
		if (phase_err_countdown <= 0) {
		  printf("%% Exiting PLL\n");
		  break; // re-initialise
		}
		phase_err_countdown--;

	      } else {

		phase_err_countdown = INIT_PHASE_ERR_COUNTDOWN;

		phase_err_int += (double)phase_err;

		double pwm_duty_cycle = 0.5 +
		  0.5 * ((double)phase_err * P_factor
			 + phase_err_int * I_factor);

		if (pwm_duty_cycle > 1)
		  pwm_duty_cycle = 1;
		if (pwm_duty_cycle < 0)
		  pwm_duty_cycle = 0;
	
		pwm_ctrl = (uint16_t)floor(pwm_duty_cycle * 65536.0);
		pwm_ctrl_hr = (uint16_t)((int32_t)
					 (fmod(pwm_duty_cycle * 65536.0, 1.0)
					  * MEP_SF) << 8);
		pwm_ctrl_hr = (pwm_ctrl_hr + 0x180) & 0xFF00;
				

		reg_addr = (char *)epwm1_addr + EPWM_CMPA;
		WR_REG16(reg_addr, pwm_ctrl); // duty cycle
		WR_REG16((char *)epwm1_addr + EPWM_CMPAHR,
			 pwm_ctrl_hr);

	      }
	    }

	    time_t time_now = time(NULL);
	    printf("%d %d %.6g %u %u %d %d %d\n", (int)time_now,
		   phase_err, phase_err_int, pwm_ctrl, pwm_ctrl_hr,
		   gnss_pps_cap, tcxo_pps_cap, fix_status);
	    fflush(stdout);

	  }
				
	  // wait a bit
	  ts.tv_sec = 0;
	  ts.tv_nsec = 100000000;
	  nanosleep(&ts, NULL);
	}

	}

	printf("Goodbye\n");

	return 0;

}
