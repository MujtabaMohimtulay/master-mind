/* ======================================================= */
/* SECTION: includes                                       */
/* ------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

#include <unistd.h>
#include <string.h>
#include <time.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

/* --------------------------------------------------------------------------- */
/* Config settings */
/* you can use CPP flags to e.g. print extra debugging messages */
/* or switch between different versions of the code e.g. digitalWrite() in Assembler */
#define DEBUG
#undef ASM_CODE

// =======================================================
// Tunables
// PINs (based on BCM numbering)
// For wiring see CW spec: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf
// GPIO pin for green LED
#define LED 13
// GPIO pin for red LED
#define LED2 5
// GPIO pin for button
#define BUTTON 19
// =======================================================
// delay for loop iterations (mainly), in ms
// in mili-seconds: 0.2s
#define DELAY 200
// in micro-seconds: 3s
#define TIMEOUT 3000000
// =======================================================
// APP constants   ---------------------------------
// number of colours and length of the sequence
#define COLS 3
#define SEQL 3
// =======================================================

// generic constants

#ifndef TRUE
#define TRUE (1 == 1)
#define FALSE (1 == 2)
#endif

#define PAGE_SIZE (4 * 1024)
#define BLOCK_SIZE (4 * 1024)

#define INPUT 0
#define OUTPUT 1

#define LOW 0
#define HIGH 1

// =======================================================
// Wiring (see inlined initialisation routine)

#define STRB_PIN 24
#define RS_PIN 25
#define DATA0_PIN 23
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22

/* ======================================================= */
/* SECTION: constants and prototypes                       */
/* ------------------------------------------------------- */

// =======================================================
// char data for the CGRAM, i.e. defining new characters for the display

static unsigned char newChar[8] =
    {
        0b11111,
        0b10001,
        0b10001,
        0b10101,
        0b11111,
        0b10001,
        0b10001,
        0b11111,
};

/* Constants */

static const int colors = COLS;
static const int seqlen = SEQL;

static char *color_names[] = {"red", "green", "blue"};

static int *theSeq = NULL;

static int *seq1, *seq2, *cpy1, *cpy2;
/* --------------------------------------------------------------------------- */

// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices

#define PI_GPIO_MASK (0xFFFFFFC0)

static unsigned int gpiobase;
static uint32_t *gpio;

static unsigned int timebase;
static volatile uint32_t *piTime;

static int timed_out = 0;

/* ------------------------------------------------------- */
// misc prototypes

int failure(int fatal, const char *message, ...);
void waitForEnter(void);
void waitForButton(uint32_t *gpio, int button);

/* ======================================================= */
/* SECTION: hardware interface (LED, button, LCD display)  */
/* ------------------------------------------------------- */
/* low-level interface to the hardware */


/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
// void digitalWrite (uint32_t *gpio, int pin, int value);

/* set the @mode@ of a GPIO @pin@ to INPUT or OUTPUT; @gpio@ is the mmaped GPIO base address */
void pinMode(uint32_t *gpio, int pin, int mode)
{

  // We get our desired register number by obtaining the ten's place of the register. i.e. the desired register
  // for PIN 19 would be 1.

  int fSel = pin / 10;

  // We understand how many bits to shift by using the following simple arithmetic operation. We multiply the one's
  // digit of the PIN by 3 and we use that as our shift value. i.e the shift value for PIN 19 would be 9*3, that is 27.

  int shift = (pin % 10) * 3;

  // Setting takes place in the 7th register, therefore we use the 7th register to set our desired bit (after
  // computing fSel and shift) to INPUT.

  if (mode == INPUT)
  {
    // *(gpio + fSel) = *(gpio + fSel) & ~(7 << shift);
    fSel=fSel*4;
    asm(
            "\tADD R0,%[gpio],%[fSel]\n"
            "\tMOV R1, #0b111\n"
            "\tLSL R1, %[shift]\n"
            "\tBIC R1, R1\n"
            "\tAND R1, R1, R0\n"
            "\tADD R0, %[gpio],%[fSel]\n"
            "\tSTR R1, [R0]\n"
        ::[gpio] "r" (&gpio),[fSel] "r" (fSel),[shift] "r" (shift)
        :"r0","r1","cc"
    );
  }

  // We repeat the same procedure however, we change the required bits to 1 as the value of the bit during output
  // will be 1.

  else if (mode == OUTPUT)
  {
    // *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (1 << shift);
    fSel=fSel*4;
    asm(
            "\tADD R0,%[gpio],%[fSel]\n"
            "\tMOV R1, #0b111\n"
            "\tLSL R1, %[shift]\n"
            "\tBIC R1, R1\n"
            "\tAND R1, R1, R0\n"
            "\tMOV R2,#1\n"
            "\tLSL R2,%[shift]\n"
            "\tORR R1,R1,R2\n"
            "\tADD R2, %[gpio],%[fSel]\n"
            "\tSTR R1, [R2]\n"
        ::[gpio] "r" (&gpio),[fSel] "r" (fSel),[shift] "r" (shift)
        :"r0","r1","r2","cc"
    );
  }
}

/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
void writeLED(uint32_t *gpio, int led, int value)
{
  if ((led & 0xFFFFFFC0) == 0)
  {

    // We initialize our offset variable

    int offset;

    if (value == LOW)
    {
      // *(gpio + 10) = 1 << (led & 31) ;

      // The 10th register is responsible for clearing and the size of an integer is 4 bytes. Therefore, we set our offset
      // value to 10 * 4 which is 40.

      offset = 10 * 4;

    }
    else if (value == HIGH)
    {
      // *(gpio + 7) = 1 << (led & 31) ;

      // The 7th register is responsible for setting and the size of an integer is 4 bytes. Therefore, we set our offset
      // value to 7 * 4 which is 28.

      offset = 7 * 4;

    }

    asm(
        "\tADD R0,%[gpio],%[offset]\n" // We add the gpio and offest values which we get from C into register R0

        "\tMOV R1, #1\n" // We set the value of register 1 to 1

        "\tLSL R1, %[shift]\n" // We now shift our value 1 using the Logical Shift Left (LSL)

        "\tSTR R1,[R0]\n" // Stores the mode (HIGH/LOW) of R0 in register R1
        :

        // These are the values we feed into the inline assembler code.

        : [gpio] "r"(gpio),
          [offset] "r"(offset),
          [shift] "r"(led)

        // These are the registers/compilers being used.

        : "r0", "r1", "cc");
  }
  else
  {
    fprintf(stderr, "only supporting on-board pins\n");
    exit(1);
  }
}

/* read a @value@ (LOW or HIGH) from pin number @pin@ (a button device); @gpio@ is the mmaped GPIO base address */
int readButton(uint32_t *gpio, int button)
{
  // if ((button & 0xFFFFFFC0 ) == 0){
  //   if ((*(gpio + 13 ) & (1 << (button & 31))) != 0){

  //     return 1;
  //   }
  //   else {
  //     return 0;
  //   }
  // }
  // else{
  //   fprintf(stderr, "only supporting on-board pins\n");
  //   exit(1);
  // }
  if ((button & 0xFFFFFFC0) == 0)
  {

    // The 13th register is responsible for reading values and the size of an integer is 4 bytes. Therefore, we initialize
    // and set our offest value to 13 * 4 which is 52.

    int offset = 13 * 4;
    int result;
    asm(
        "\tADD R0,%[gpio],%[offset]\n" // We add the gpio and offest values which we get from C into register R0
                                       // C equivalent: gpio + 13

        "\tLDR R0, [R0]\n" // Now that we have our required PI data, we load it onto our register using the LDR
                           // pseudo instruction

        "\tMOV R1, #1\n" // We set the value of register 1 to 1 (which is what the button should return when pressed)

        "\tLSL R1, %[shift]\n" // We now shift our value 1 using the Logical Shift Left (LSL)
                               // C equivalent: 1 << (button & 31)

        "\tAND %[result], R1, R0\n" // The AND instruction compares and evaluates the values present in R1 and R0 and
                                    // stores them in %[result] which we pass on to C.

        : [result] "=r"(result) // This is the value we get from the inline assembler code.

        // These are the values we feed into the inline assembler code.

        : [gpio] "r"(gpio),
          [offset] "r"(offset),
          [shift] "r"(button)

        // These are the registers/compilers being used.

        : "r0", "r1", "cc");

    return result;
  }
  else
  {
    fprintf(stderr, "only supporting on-board pins\n");
    exit(1);
  }
}

/* wait for a button input on pin number @button@; @gpio@ is the mmaped GPIO base address */
void waitForButton(uint32_t *gpio, int button);

/* ======================================================= */
/* SECTION: game logic                                     */
/* ------------------------------------------------------- */
/* AUX fcts of the game logic */


/* initialise the secret sequence; by default it should be a random sequence */
void initSeq()
{

  // We initialise the random number generator, using the time as the seed

  srand(time(NULL));

  // We malloc and instantiate our array theSeq

  theSeq = (int *)malloc(seqlen * sizeof(int));

  for (int i = 0; i < seqlen; i++)
  {

    // We add the randomly generated value as an element to the array. As per our coursework specs and the following code,
    // the randomly generated numbers will range between only 1 to 3.

    theSeq[i] = (rand() % 3) + 1;
  }
}

/* display the sequence on the terminal window, using the format from the sample run in the spec */
void showSeq(int *seq)
{
  fprintf(stdout, "The secret sequence is: %d %d %d\n", seq[0], seq[1], seq[2]);
}

#define NAN1 8
#define NAN2 9

/* counts how many entries in seq2 match entries in seq1 */
/* returns exact and approximate matches, either both encoded in one value, */
/* or as a pointer to a pair of values */
int /* or int* */ countMatches(int *seq1, int *seq2)
{
  /* ***  COMPLETE the code here  ***  */

  // We instantiate the matchesFound variable

  int matchesFound;

  // We instantiate and set the counter of our exact number of guesses to 0

  int exact = 0;

  // We instantiate and set the counter of our approximate number of guesses to 0

  int approximate = 0;

  // We malloc and initialize an array which we use to verify whether the element in-hand has already been checked or not

  int *checked = malloc(3 * sizeof(int));

  // As per our coursework specs, all our sequences will have only 3 elements. Therefore, the loop will also iterate
  // only 3 times.

  for (int i = 0; i < 3; i++)
  {
    // If the element-in-hand of the user-entered sequence is equal to the corresponding element-in-hand of the
    // system-generated squence, then this means that its an exact match. As both, the value as well as the position of
    // the number is correct.

    if (seq1[i] == seq2[i])
    {
      // Therefore, we increment our exact counter variable by 1.
      exact++;

      // However, this element could have already been checked beforehand and be deemed as an approximate guess. So, we
      // verify with our array checked to see whether or not the element-in-hand has already been accounted for.

      // If yes:

      if (checked[i])
      {

        // We decrement the approximate guess counter by 1 as this is not an approximate guess, it is an exact one.

        approximate--;
      }

      // We change the value of the corresponding element in the checked array to 1 so as to signify that the
      // element-in-hand has been checked.

      checked[i] = 1;
    }

    // If the sequence entered by the user is not exactly the same as the system-generated-sequence:

    else if (seq1[i] != seq2[i])
    {

      // Another for loop is ran within the system-generated-sequence. This for loop is to check if the element-in-hand
      // entered by the user exists somewhere in the system-generated-sequence if not in the exact location.

      for (int j = 0; j < 3; j++)
      {
        if (seq1[j] == seq2[i])
        {

          // If it does exist elsewhere in the system-generated-sequence, then we increment the approximate counter by 1
          // as the value is correct but the position is wrong. We also insert 1 into the corresponding element in our
          // list checked so as to signify that the element-in-hand has been checked.

          if (!checked[j])
          {
            approximate++;
            checked[j] = 1;

            // We break so that it repeats the same process for the next element in the user-entered list and so on and
            // so forth.

            break;
          }
        }
      }
    }
  }

  // When returning, we return it as a single number with the exact number of matches at the ten's place and the
  // approximate number of matches at the one's.

  matchesFound = (exact * 10) + approximate;
  return matchesFound;
}

/* show the results from calling countMatches on seq1 and seq1 */

/* show the results from calling countMatches on seq1 and seq1 */
void showMatches(int  code,  int *seq1, int *seq2,  int lcd_format)
{
  

  // We are displaying our exact and approximate matches as one number.
  // For instance, if there are 3 exact matches and 0 approximate matches, we display the output as 30.
  // The exact number of matches occupy the ten's place and the approximate number of matches occupy the one's place.

  // We divide by 10 to get the 10's place.

  int exactMatch = code / 10;

  // We mod by 10 to get the one's place.

  int approxMatch = code % 10;

  // Then we print onto terminal the number of exact and approximate matches.
  printf("%d exact\n", exactMatch);
  printf("%d approximate\n", approxMatch);
}

/* parse an integer value as a list of digits, and put them into @seq@ */
/* needed for processing command-line with options -s or -u            */
void readSeq(int *seq, int val)
{
 

  // We implement the following function for testing purposes.
  // The goal is to break a given number down and insert each digit of the said number as different elements in the
  // array seq.

  // We assign the value to j to further operate on.

  int j = val;

  // Our specs demand only 3 inputs, there the number of elements present in the array will always be 3.

  for (int i = 2; i >= 0; i--)
  {

    // We assign each element in the array to the corresponding digit starting from the one's place to the hundred's
    // place. For instance if the number entered was 123, the first element to be added in the list would be 3 followed
    // by 2 and then followed by 1.

    // We do this by segregating the one's place by modding our number by 10 and adding that number to the array.

    seq[i] = (j % 10);

    // We then get rid of the one's digit by dividing the number by 10. Hence, in the next iteration of this loop there
    // will be only 2 digits. After segregating and eliminating the digit once again, there will be only one digit
    // remaining.

    // This way, we ensure that all digits are iterated through and are inserted successfully into our array.

    j = j / 10;
  }
}


/* ======================================================= */
/* SECTION: TIMER code                                     */
/* ------------------------------------------------------- */
/* TIMER code */

/* timestamps needed to implement a time-out mechanism */
static uint64_t startT, stopT;

// from tutorial codes

uint64_t timeInMicroseconds()
{

  // We make use of the timeval struct and call it as tv. This struct is responsible storing the amount
  // of time. The struct comprises of tv_sec and tv_usec which is used to store the number of seconds and micro-seconds
  // respectively that have passed since the event-in-hand took place.

  struct timeval tv;

  // We initialize our unsigned integer variable start which will keep count of our program execution time.

  uint64_t start;

  // We use the gettimeofday function to do just that. The first argument passed in the function points to our timeval
  // struct and the second argument is supposed to signify the time zone. However, it is no longer required and is now
  // obsolete so therefore, it has been set to NULL.

  gettimeofday(&tv, NULL);

  // We calculate the amount of time that has passed since the function has been called and store it in our unsigned
  // integer variable start.

  start = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)tv.tv_usec;

  // We return the required time.

  return (uint64_t)start;
}


/* ======================================================= */
/* SECTION: Aux function                                   */
/* ------------------------------------------------------- */
/* misc aux functions */

int failure(int fatal, const char *message, ...)
{
  va_list argp;
  char buffer[1024];

  if (!fatal) //  && wiringPiReturnCodes)
    return -1;

  va_start(argp, message);
  vsnprintf(buffer, 1023, message, argp);
  va_end(argp);

  fprintf(stderr, "%s", buffer);
  exit(EXIT_FAILURE);

  return 0;
}

/*
 * waitForEnter:
 *********************************************************************************
 */

void waitForEnter(void)
{
  printf("Press ENTER to continue: ");
  (void)fgetc(stdin);
}

/*
 * delay:
 *	Wait for some number of milliseconds
 *********************************************************************************
 */

void delay(unsigned int howLong)
{
  struct timespec sleeper, dummy;

  sleeper.tv_sec = (time_t)(howLong / 1000);
  sleeper.tv_nsec = (long)(howLong % 1000) * 1000000;

  nanosleep(&sleeper, &dummy);
}


void delayMicroseconds(unsigned int howLong)
{
  struct timespec sleeper;
  unsigned int uSecs = howLong % 1000000;
  unsigned int wSecs = howLong / 1000000;

  /**/ if (howLong == 0)
    return;
#if 0
  else if (howLong  < 100)
    delayMicrosecondsHard (howLong) ;
#endif
  else
  {
    sleeper.tv_sec = wSecs;
    sleeper.tv_nsec = (long)(uSecs * 1000L);
    nanosleep(&sleeper, NULL);
  }
}

/* ======================================================= */
/* SECTION: aux functions for game logic                   */
/* ------------------------------------------------------- */

/* interface on top of the low-level pin I/O code */

/* blink the led on pin @led@, @c@ times */
void blinkN(uint32_t *gpio, int led, int c)
{

  // This method is responsible for blinking either of the leds (red or green) depending on the argument passed (the
  // corresponding argument is int led) for n number of times based on the argument passed (the corresponding argument is
  // int c).

  // For instance, if we want to make the red LED blink twice, we'd call this method as blinkN(gpio, 5, 2) as 5 is the PIN
  // number of the red LED and 2 as we want it to blink twice.

  for (int i = 0; i < c; i++)
  {

    // We turn the LED on by setting our desired PIN to HIGH using our writeLED method.

    writeLED(gpio, led, HIGH);

    // We've added some delay before the next blink to avoid it from blinking too fast and to execute our program flow
    // at a healthy pace.

    {
      struct timespec sleeper, dummy;

      sleeper.tv_sec = (time_t)(200 / 1000);
      sleeper.tv_nsec = (long)(200 % 1000) * 1000000;

      nanosleep(&sleeper, &dummy);
    }

    // We turn the LED off by setting our desired PIN to LOW using our writeLED method.

    writeLED(gpio, led, LOW);

    // We've added some delay before the next blink to avoid it from blinking too fast and to execute our program flow
    // at a healthy pace.

    {
      struct timespec sleeper, dummy;

      sleeper.tv_sec = (time_t)(200 / 1000);
      sleeper.tv_nsec = (long)(200 % 1000) * 1000000;

      nanosleep(&sleeper, &dummy);
    }
    // This proccess ensues until the method has finished iterating completely.
  }
}

/* ======================================================= */
/* SECTION: main fct                                       */
/* ------------------------------------------------------- */

int main(int argc, char *argv[])
{ // this is just a suggestion of some variable that you may want to use
  int bits, rows, cols;
  unsigned char func;

  int found = 0, attempts = 0, i, j, code;
  int c, d, buttonPressed, rel, foo;
  int *attSeq;

  int pinLED = LED, pin2LED2 = LED2, pinButton = BUTTON;
  int fSel, shift, pin, clrOff, setOff, off, res;
  int fd;

  int exact, contained;
  char str1[32];
  char str2[32];

  struct timeval t1, t2;
  int t;

  char buf[32];

  // variables for command-line processing
  char str_in[20], str[20] = "some text";
  int verbose = 0, debug = 0, help = 0, opt_m = 0, opt_n = 0, opt_s = 0, unit_test = 0, res_matches = 0;

  // -------------------------------------------------------
  // process command-line arguments

  // see: man 3 getopt for docu and an example of command line parsing
  { 
    int opt;
    while ((opt = getopt(argc, argv, "hvdus:")) != -1)
    {
      switch (opt)
      {
      case 'v':
        verbose = 1;
        break;
      case 'h':
        help = 1;
        break;
      case 'd':
        debug = 1;
        break;
      case 'u':
        unit_test = 1;
        break;
      case 's':
        opt_s = atoi(optarg);
        break;
      default: /* '?' */
        fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]  \n", argv[0]);
        exit(EXIT_FAILURE);
      }
    }
  }

  if (help)
  {
    fprintf(stderr, "MasterMind program, running on a Raspberry Pi, with connected LED, button and LCD display\n");
    fprintf(stderr, "Use the button for input of numbers. The LCD display will show the matches with the secret sequence.\n");
    fprintf(stderr, "For full specification of the program see: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf\n");
    fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]  \n", argv[0]);
    exit(EXIT_SUCCESS);
  }

  if (unit_test && optind >= argc - 1)
  {
    fprintf(stderr, "Expected 2 arguments after option -u\n");
    exit(EXIT_FAILURE);
  }

  if (verbose && unit_test)
  {
    printf("1st argument = %s\n", argv[optind]);
    printf("2nd argument = %s\n", argv[optind + 1]);
  }

  if (verbose)
  {
    fprintf(stdout, "Settings for running the program\n");
    fprintf(stdout, "Verbose is %s\n", (verbose ? "ON" : "OFF"));
    fprintf(stdout, "Debug is %s\n", (debug ? "ON" : "OFF"));
    fprintf(stdout, "Unittest is %s\n", (unit_test ? "ON" : "OFF"));
    if (opt_s)
      fprintf(stdout, "Secret sequence set to %d\n", opt_s);
  }

  seq1 = (int *)malloc(seqlen * sizeof(int));
  seq2 = (int *)malloc(seqlen * sizeof(int));
  cpy1 = (int *)malloc(seqlen * sizeof(int));
  cpy2 = (int *)malloc(seqlen * sizeof(int));

  // check for -u option, and if so run a unit test on the matching function
  if (unit_test && argc > optind + 1)
  { // more arguments to process; only needed with -u
    strcpy(str_in, argv[optind]);
    opt_m = atoi(str_in);
    strcpy(str_in, argv[optind + 1]);
    opt_n = atoi(str_in);
    // CALL a test-matches function; see testm.c for an example implementation
    readSeq(seq1, opt_m); // turn the integer number into a sequence of numbers
    readSeq(seq2, opt_n); // turn the integer number into a sequence of numbers
    if (verbose)
      fprintf(stdout, "Testing matches function with sequences %d and %d\n", opt_m, opt_n);
    res_matches = countMatches(seq1, seq2);
    showMatches(res_matches, seq1, seq2, 1);
    exit(EXIT_SUCCESS);
  }
  else
  {
    /* nothing to do here; just continue with the rest of the main fct */
  }

  if (opt_s)
  { // if -s option is given, use the sequence as secret sequence
    if (theSeq == NULL)
      theSeq = (int *)malloc(seqlen * sizeof(int));
    readSeq(theSeq, opt_s);
    if (verbose)
    {
      fprintf(stderr, "Running program with secret sequence:\n");
      showSeq(theSeq);
    }
  }

  // -------------------------------------------------------
  
  // -------------------------------------------------------

  

  if (geteuid() != 0)
    fprintf(stderr, "setup: Must be root. (Did you forget sudo?)\n");

  // init of guess sequence, and copies (for use in countMatches)
  attSeq = (int *)malloc(seqlen * sizeof(int));
  cpy1 = (int *)malloc(seqlen * sizeof(int));
  cpy2 = (int *)malloc(seqlen * sizeof(int));

  // -----------------------------------------------------------------------------
  // constants for RPi2
  gpiobase = 0x3F200000;
  timebase = 0x3F003000;
  // -----------------------------------------------------------------------------
  // memory mapping
  // Open the master /dev/memory device

  if ((fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0)
    return failure(FALSE, "setup: Unable to open /dev/mem: %s\n", strerror(errno));

  // GPIO:
  gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, gpiobase);
  if ((int32_t)gpio == -1)
    return failure(FALSE, "setup: mmap (GPIO) failed: %s\n", strerror(errno));

  piTime = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, timebase);
  

  /* initialise the secret sequence */
  if (!opt_s)
    initSeq();
  if (debug)
    showSeq(theSeq);

  // optionally one of these 2 calls:
  waitForEnter () ;
  // waitForButton (gpio, pinButton) ;

  // -----------------------------------------------------------------------------
  // +++++ main loop

  // We set up the modes of our hardwares as follows.

  // Since our LEDs emit light, we set them to OUTPUT and as users enter values through the button, we set that to INPUT.

  pinMode(gpio, pinLED, OUTPUT);
  pinMode(gpio, pin2LED2, OUTPUT);
  pinMode(gpio, pinButton, INPUT);

  while (!found)
  {
    attempts++;

    // This gets omitted if it is still only the first round being played, however the program goes into this for every
    // other round being played.

    if (attempts > 1)
    {
      // According to the Game Functionality section from the coursework specs, the red control LED should blink three
      // times to indicate the start of a new round.

      // Therefore, we are doing exactly that with the help of the blinkN() method.

      printf("Try Again!\n");
      blinkN(gpio, pin2LED2, 3);
    }

    // There will be a total of 3 rounds that will be played, therefore we implement a for loop with 3 iterations.

    for (int i = 0; i < 3; i++)
    {
      int count = 0;
      {
        volatile uint32_t ts = *(piTime + 1); // word offset
        printf("Enter Digit %d \n",i+1);

        // As long as the button is pressed before 3 seconds are up:

        while ((*(piTime + 1) - ts) < TIMEOUT)
        {
          if (readButton(gpio, pinButton))
          {

            // The number of times it is pressed is counted.

            count++;
            fprintf(stdout,"1");
          }

          // If not, the timer is reset and the user gets to press the button again. This way, we ensure that we dont
          // carry forward a button input of 0 in the game. The user is forced to press the button atleast once, else
          // the program won't advance further.

          if (count == 0)
          {
            ts = *(piTime + 1);
          }
        }
        printf("\n");
      }

      // We record the user-inputs in the array attSeq

      attSeq[i] = count;

      // We add a delay in order to prevent the flow of execution from occuring too fast.

      delayMicroseconds(1000000);

      // We blink the red LED once to acknowledge the input of the number followed by blinking the green LED as many
      // times as the button was pressed.

      blinkN(gpio, pin2LED2, 1);
      blinkN(gpio, pinLED, count);
    }

    // We add a delay in order to prevent the flow of execution from occuring too fast.

    delayMicroseconds(2000000);

    // Once all values have been entered and echoed, the red control LED is blinked twice to indicate the end of the input.

    blinkN(gpio, pin2LED2, 2);
    int valid = 0;

    // We go through a screening process just to make sure that there aren't any 0 (invalid) entries as this is not
    // allowed in our game. We increment a counter for every element that we validate.

    for (int k = 0; k < 3; k++)
    {
      if (attSeq[i] != 0)
      {
        valid++;
      }
    }

    // Once we've finished validation of all 3 elements, we get the results using the countMatches method.

    if (valid == 3)
    {
      int result = countMatches(theSeq, attSeq);
      if(debug){
        showMatches(result,theSeq,attSeq, 1);
      }
      // If the result was 30, this would mean that the user has guessed the system-generated sequence correctly with
      // 3 exact matches and 0 approximate ones.

      if (result == 30)
      {
        // We first make the green LED blink the number of exact matches.

        blinkN(gpio, pinLED, (result / 10));

        // We then add a delay in order to prevent the flow of execution from occuring too fast.

        delayMicroseconds(1000000);

        // To serve as a separator, we blink the red LED once.

        blinkN(gpio, pin2LED2, 1);

        // We then add a delay in order to prevent the flow of execution from occuring too fast.

        delayMicroseconds(1000000);

        // Finally, we make the green LED blink the number of approximate matches.

        blinkN(gpio, pinLED, (result % 10));

        // Followed by a delay in order to prevent the flow of execution from occuring too fast.

        delayMicroseconds(1000000);
               

        // We increment our found counter to signify that the user has completed the game successfully.

        found++;
        delayMicroseconds(500000);
      }

      // If the result is something else, then this would mean that the user did not guess the sequence correctly.
      // If this is the case

      else if ((result % 10) != 0 || (result / 10) != 0)
      {

        // We first make the green LED blink the number of exact matches.

        blinkN(gpio, pinLED, (result / 10));

        // We then add a delay in order to prevent the flow of execution from occuring too fast.

        delayMicroseconds(1000000);

        // To serve as a separator, we blink the red LED once.

        blinkN(gpio, pin2LED2, 1);

        // We then add a delay in order to prevent the flow of execution from occuring too fast.

        delayMicroseconds(1000000);

        // Finally, we make the green LED blink the number of approximate matches.

        blinkN(gpio, pinLED, (result % 10));

        // Followed by a delay in order to prevent the flow of execution from occuring too fast.

        delayMicroseconds(1000000);
        // showMatches(result);
      }
    }

  }

  // If the result was 30, the user has completed the game successfully and the program thus goes into the following if
  // condition:

  if (found)
  {
    /* ***  COMPLETE the code here  ***  */
    printf("You guessed the sequence correctly!\n");
    printf("You took %d attempts!\n\n", attempts);

    // We make the green LED blink three times while the red LED is turned on in order to represent the end of the game.

    writeLED(gpio, pin2LED2, HIGH);
    blinkN(gpio, pinLED, 3);
    writeLED(gpio, pin2LED2, LOW);

    printf("Thank you for playing Mastermind! Have a great day :)\n");
    exit(0);
  }
  else
  {
    fprintf(stdout, "Sequence not found\n");
  }
  return 0;
}
