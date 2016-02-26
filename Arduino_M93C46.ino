/*
    An application to communicate with an M93C46 EEPROM part. This application is designed to run on an Arduino Uno with some attached
    circuitry on a proto board or breadboard. I have written a blog post that describes the basics of SPI and introduces this code.
    See:

    http://geeklikemetoo.blogspot.com/2016/02/spi-communications-with-arduino-uno-and.html

    - Four I/O lines are connected to pins on the EEPROM.

    - Three I/O pins are used to flash three LEDs indicating busy (yellow or blue), success (green), or failure (red). These also have
      paths to GND.

    The M93C46 part uses a "Microwire" communication protocol that is a variation of SPI.

    The regular Arduino libraries do not support the irregular data lengths used by this part, so we just bit-bang the clock, data out,
    and slave select lines, and use the Arduino standard digitalRead() call to determine the state of the incoming data line.
    
    This program does not exercise the entire command set. I have tested ERASE commands but not the WRITE ALL or ERASE ALL commands.
    They should work, but if you add code to use these commands, make sure to test it carefully.
    
    We are writing very small amounts of data, and getting it correct was more important to me than making it fast, so I used hard-coded
    1 msec delays between transitions of the clock and data signals. This makes the actual clock speed of a single command about 500 Hz,
    with a 3 msec margin around the command for bringing the slave select up and down.

    I am not using the status feature which involves raising the slave select and reading the MISO pin to detect when the read or write
    cycle has finished. Instead I just read back the data to verify. The status feature might be useful if we needed to drive a series of
    writes at a rate closer to the maximum supported speed.
*/

/*
    Data pins
*/
#define LED_RED      2
#define LED_GREEN    3
#define LED_BLUE     4

#define SLAVESELECT 10 /* SS   */
#define SPICLOCK    11 /* SCK  */
#define DATAOUT     12 /* MOSI */
#define DATAIN      13 /* MISO */

/*
    The command set for an M93C46 part with ORG = 1 (pulled up). This gives us 16-bit memory organization,
    so the 1 Kbit (1024 bits) divided by 16 gives us 64 or 0x40 words, with addresses 0..63 or 0x0..3F
    (addresses are 6 bits wide).

    Note that these are only valid for the 1 Kb part. If we wanted to support the range of parts with different
    sizes we would probably want to generate these programmatically.

    The constants below define the commands. In these definitions the "don't care" X bits, data bits, and
    address bits left as zero. The code will fill in the address and data bits as needed.

    Note! On the Arduino, you must define these using the UL suffix, to generate an unsigned long, rather than
    unsigned int, type. The basic Arduino Uno uses 16-bit ints. Since the Arduino IDE turns off all compilation
    warnings, defining constants that are too big to fit into an int will not even generate a warning -- the
    extra bits past bit 15 will just be silently discarded.
*/
#define CMD_16_WDS   ( 16UL <<  4 ) /* 1 00 00 XXXX                   Write Disable (9  bits:   1 start bit, 2 opcode bits, 2 extended opcode bits, 4 bits are "don't care")               */
#define CMD_16_WRAL  ( 17UL << 20 ) /* 1 00 01 XXXX DDDDDDDD DDDDDDDD Write All     (25 bits:   1 start bit, 2 opcode bits, 2 extended opcode bits, 4 bits are "don't care", 16 data bits) */
#define CMD_16_ERAL  ( 18UL <<  4 ) /* 1 00 10 XXXX                   Erase All     (9  bits:   like WDS)                                                                                  */
#define CMD_16_WEN   ( 19UL <<  4 ) /* 1 00 11 XXXX                   Write Enable  (9  bits:   like WDS)                                                                                  */
#define CMD_16_WRITE ( 5UL  << 22 ) /* 1 01 AA AAAA DDDDDDDD DDDDDDDD Write         (25 bits:   1 start bit, 2 opcode bits, 6 address bits, 16 data bits)                                  */

/*
    Note: for the read command constant, we leave off the dummy outgoing bits used to read the value; these are handled by a separate read function
*/
#define CMD_16_READ  ( 6UL  <<  6 ) /* 1 10 AA AAAA XXXXXXXX XXXXXXXX Read          (9  bits+*: like WRITE, outgoing data bits for read are "don't care")                                  */
#define CMD_16_ERASE ( 7UL  <<  6 ) /* 1 11 AA AAAA                   Erase         (9  bits:   1 start bit, 2 opcode bits, 6 address bits).                                               */

/*
    *Note: for reading, after sending the address, when we start clocking out our "don't care" bits, we get back an initial zero dummy bit,
    so to read all the bits we need to clock out our 16 "don't care" bits plus one extra "don't care" bit. We could actually clock in multiple
    words in which case the chip returns the values from consecutive addresses (with rollover), although my read function does not currently
    support this. Words read after the first one don't have the leading dummy zero bit.
*/
#define CMD_16_WDS_NUM_BITS   (  9 )
#define CMD_16_WRAL_NUM_BITS  ( 25 )
#define CMD_16_ERAL_NUM_BITS  (  9 )
#define CMD_16_WEN_NUM_BITS   (  9 )
#define CMD_16_WRITE_NUM_BITS ( 25 )

/*
    Does not include the bits clocked out to read the returned value
*/
#define CMD_16_READ_NUM_BITS  (  9 )
#define CMD_16_ERASE_NUM_BITS (  9 )


/*
    The command set for an M93C46 part with ORG = 0 (pulled down). This gives us 8-bit memory organization,
    so the 1 Kbit (1024 bits) divided by 8 gives us 128 or 0x80 words, with addresses 0..127 or 0x0..7F
    (addresses are 7 bits wide).
*/
#define CMD_8_WDS   ( 16UL <<  5 ) /* 1 00 00X XXXX          Write Disable (10 bits:    1 start bit, 2 opcode bits, 2 extended opcode bits, 5 bits are "don't care")              */
#define CMD_8_WRAL  ( 17UL << 13 ) /* 1 00 01X XXXX DDDDDDDD Write All     (18 bits:    1 start bit, 2 opcode bits, 2 extended opcode bits, 5 bits are "don't care", 8 data bits) */
#define CMD_8_ERAL  ( 18UL <<  5 ) /* 1 00 10X XXXX          Erase All     (10 bits:    like WDS)                                                                                 */
#define CMD_8_WEN   ( 19UL <<  5 ) /* 1 00 11X XXXX          Write Enable  (10 bits:    like WDS)                                                                                 */
#define CMD_8_WRITE ( 5UL  << 15 ) /* 1 01 AAA AAAA DDDDDDDD Write         (18 bits:    1 start bit, 2 opcode bits, 6 address bits, 16 data bits)                                 */

/*
    Note: for the read command constant, we leave off the dummy outgoing bits used to read the value
*/
#define CMD_8_READ  ( 6UL  <<  7 ) /* 1 10 AAA AAAA XXXXXXXX Read          (10 bits+**: like WRITE, outgoing data bits for read are "don't care")                                 */
#define CMD_8_ERASE ( 7UL  <<  7 ) /* 1 11 AAA AAAA          Erase         (10 bits:    1 start bit, 2 opcode bits, 6 address bits).                                              */


/*
    **Note: for reading, after sending the address, when we start clocking out our first "don't care" bit, we get back an initial zero dummy bit,
    so to read all the bits we need to clock out our 8 "don't care" bits plus one extra "don't care" bit. We could actually clock in multiple
    bytes in which case the chip returns the values from consecutive addresses (with rollover), although my read function does not currently
    support this. Bytes read after the first one don't have the leading dummy zero bit.
*/
#define CMD_8_WDS_NUM_BITS   ( 10 )
#define CMD_8_WRAL_NUM_BITS  ( 18 )
#define CMD_8_ERAL_NUM_BITS  ( 10 )
#define CMD_8_WEN_NUM_BITS   ( 10 )
#define CMD_8_WRITE_NUM_BITS ( 18 )

/*
    Does not include the bits clocked out to read the returned value
*/
#define CMD_8_READ_NUM_BITS  ( 10 )

#define CMD_8_ERASE_NUM_BITS ( 10 )

/*
    Simple functions to insert values into the commands defined above
*/
uint32_t assemble_CMD_16_WRAL (               uint16_t val ) { return ( uint32_t )CMD_16_WRAL                               | ( uint32_t )val; }
uint32_t assemble_CMD_16_WRITE( uint8_t addr, uint16_t val ) { return ( uint32_t )CMD_16_WRITE | ( ( uint32_t )addr << 16 ) | ( uint32_t )val; }
uint32_t assemble_CMD_8_WRITE( uint8_t addr,  uint8_t val  ) { return ( uint32_t )CMD_8_WRITE  | ( ( uint32_t )addr <<  8 ) | ( uint32_t )val; }

/*
    For the read command, only encode the command bits and address bits; the read functions will clock out the extra bits required to read
    in the data.
 */
uint32_t assemble_CMD_16_READ ( uint8_t addr               ) { return ( uint32_t )CMD_16_READ  | ( ( uint32_t )addr )                        ; }
uint32_t assemble_CMD_8_READ  ( uint8_t addr               ) { return ( uint32_t )CMD_8_READ   | ( ( uint32_t )addr )                        ; }

uint32_t assemble_CMD_16_ERASE( uint8_t addr               ) { return ( uint32_t )CMD_16_ERASE | ( ( uint32_t )addr )                        ; }


/*
    Do a simple bit-bang SPI transfer with hard-coded delays.

    Clock is low when idle. The data line is sampled on the leading edge of the
    clock, so we change it on the falling edge. Have the LEDs follow the slave select,
    so we can see that something is happening. Select should be high when active.
*/
#define SLAVE_SEL_DELAY_PRE_CLOCK_MSEC    ( 3 )
#define SLAVE_SEL_DELAY_POST_CLOCK_MSEC   ( 3 )
#define INTER_CLOCK_TRANSITION_DELAY_MSEC ( 1 )

void write_bit_series( uint32_t bits, uint8_t num_bits_to_send )
{
    uint8_t num_bits_sent;

    for ( num_bits_sent = 0; num_bits_sent < num_bits_to_send; num_bits_sent += 1 )
    {
        digitalWrite( SPICLOCK, LOW );
        digitalWrite( DATAOUT, bits & ( 1UL << ( num_bits_to_send - num_bits_sent - 1 ) ) ? HIGH : LOW );
        delay( INTER_CLOCK_TRANSITION_DELAY_MSEC );
        digitalWrite( SPICLOCK, HIGH );
        delay( INTER_CLOCK_TRANSITION_DELAY_MSEC );

    }

}


/*
    Use this function to send any command other than a read command
*/
void write_cmd( uint32_t bits, uint8_t num_bits_to_send )
{
    digitalWrite( LED_BLUE, HIGH );
    digitalWrite( SLAVESELECT, HIGH );
    delay ( SLAVE_SEL_DELAY_PRE_CLOCK_MSEC );

    write_bit_series( bits, num_bits_to_send );

    /*
        Leave the data and clock lines low after the last bit sent
     */
    digitalWrite( DATAOUT, LOW );
    digitalWrite( SPICLOCK, LOW );

    delay ( SLAVE_SEL_DELAY_POST_CLOCK_MSEC );
    digitalWrite( SLAVESELECT, LOW );
    digitalWrite( LED_BLUE, LOW );

}


/*
    Like the write, except that we also sample the MISO pin on the falling clock edges.
    Incoming data bits are are available starting after the last address bit is sent, and
    an additional clock cycle to read in an initial dummy zero bit. Use the blue LED to
    indicate chip select for read operations.
*/
uint16_t read_16( uint8_t addr )
{
    uint8_t num_bits_to_read = 16;
    uint16_t in_bits = 0;
    uint32_t out_bits = assemble_CMD_16_READ( addr );

    digitalWrite( LED_BLUE, HIGH );
    digitalWrite( SLAVESELECT, HIGH );
    delay ( SLAVE_SEL_DELAY_PRE_CLOCK_MSEC );

    /*
        Write out the read command and address
    */
    write_bit_series( out_bits, CMD_16_READ_NUM_BITS );

    /*
        Insert an extra clock to handle the incoming dummy zero bit
    */
    digitalWrite( DATAOUT, LOW );

    digitalWrite( SPICLOCK, LOW );
    delay( 1 );

    digitalWrite( SPICLOCK, HIGH );
    delay( 1 );

    /*
        Now read 16 bits by clocking. Leave the outgoing data line low. The incoming data line should change on the rising edge of the
        clock, so read it on the falling edge.
    */
    for ( ; num_bits_to_read > 0; num_bits_to_read -= 1 )
    {
        digitalWrite( SPICLOCK, LOW );
        uint16_t in_bit = ( ( HIGH == digitalRead( DATAIN ) ) ? 1UL : 0UL );
        in_bits |= ( in_bit << ( num_bits_to_read - 1 ) );
        delay( INTER_CLOCK_TRANSITION_DELAY_MSEC );

        digitalWrite( SPICLOCK, HIGH );
        delay( INTER_CLOCK_TRANSITION_DELAY_MSEC );
    }

    /*
        Leave the data and clock lines low after the last bit sent
     */
    digitalWrite( DATAOUT, LOW );
    digitalWrite( SPICLOCK, LOW );

    delay ( SLAVE_SEL_DELAY_POST_CLOCK_MSEC );
    digitalWrite( SLAVESELECT, LOW );
    digitalWrite( LED_BLUE, LOW );

    return in_bits;

}


/*
    A similar function for handling 8-bit reads
*/
uint16_t read_8( uint8_t addr )
{
    uint8_t num_bits_to_read = 8;
    uint8_t in_bits = 0;
    uint32_t out_bits = assemble_CMD_8_READ( addr );

    digitalWrite( LED_BLUE, HIGH );
    digitalWrite( SLAVESELECT, HIGH );
    delay ( SLAVE_SEL_DELAY_PRE_CLOCK_MSEC );

    /*
        Write out the read command and address
    */
    write_bit_series( out_bits, CMD_8_READ_NUM_BITS );

    /*
        Insert an extra clock to handle the incoming dummy zero bit
    */
    digitalWrite( DATAOUT, LOW );

    digitalWrite( SPICLOCK, LOW );
    delay( 1 );

    digitalWrite( SPICLOCK, HIGH );
    delay( 1 );

    /*
        Now read 8 bits by clocking. Leave the outgoing data line low. The incoming data line should change on the rising edge of the
        clock, so read it on the falling edge.
    */
    for ( ; num_bits_to_read > 0; num_bits_to_read -= 1 )
    {
        digitalWrite( SPICLOCK, LOW );
        uint8_t in_bit = ( ( HIGH == digitalRead( DATAIN ) ) ? 1UL : 0UL );
        in_bits |= ( in_bit << ( num_bits_to_read - 1 ) );
        delay( INTER_CLOCK_TRANSITION_DELAY_MSEC );

        digitalWrite( SPICLOCK, HIGH );
        delay( INTER_CLOCK_TRANSITION_DELAY_MSEC );
    }

    /*
        Leave the data and clock lines low after the last bit sent
     */
    digitalWrite( DATAOUT, LOW );
    digitalWrite( SPICLOCK, LOW );

    delay ( SLAVE_SEL_DELAY_POST_CLOCK_MSEC );
    digitalWrite( SLAVESELECT, LOW );
    digitalWrite( LED_BLUE, LOW );

    return in_bits;

}


void setup()
{
    Serial.begin(9600);

    pinMode( LED_RED, OUTPUT );
    pinMode( LED_GREEN, OUTPUT );
    pinMode( LED_BLUE, OUTPUT );

    pinMode( DATAOUT, OUTPUT );
    pinMode( SPICLOCK,OUTPUT );
    pinMode( SLAVESELECT,OUTPUT );

    pinMode( DATAIN, INPUT );

    /*
        Initially, select is high (device is not selected; clock is low; data is "don't care"
    */
    digitalWrite(SLAVESELECT,HIGH);
    digitalWrite( SPICLOCK, LOW );

    /*
        Initially, our signalling LEDs are off
    */
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);

}


void flash_result( int pin )
{
    digitalWrite( pin, HIGH );
    delay( 100 );
    digitalWrite( pin, LOW );
    delay( 100 );
    digitalWrite( pin, HIGH );
    delay( 100 );
    digitalWrite( pin, LOW );
    delay( 100 );
    digitalWrite( pin, HIGH );
    delay( 100 );
    digitalWrite( pin, LOW );
    delay( 100 );

}


void loop()
{
    #define TEST_READ_WRITE_16     ( 1 )
    #define TEST_READ_WRITE_8      ( 2 )

    #define RUN_AS ( TEST_READ_WRITE_8 )

    #if ( RUN_AS == TEST_READ_WRITE_16 )

        /*
            NOTE: to operate in 16-bit mode, the EEPROM's ORG pin must be connected
            to the positive supply VCC (in my testing, +3.3V or +5). According to the
            documentation it can also be left to float, but in my testing this did not
            work. 

            Enable writing to the EEPROM, using the 16-bit write enable command.
        */
        write_cmd( ( uint16_t )CMD_16_WEN, CMD_16_WEN_NUM_BITS );
        delay( 25 );

        while ( 1 )
        {
            uint8_t addr;
            uint16_t read_val;
            uint16_t write_val;
    
            for ( addr = 0x0; addr <= 0x3F; addr++ )
            {
                write_val = ~( uint16_t )random( 65536 );
    
                write_cmd( assemble_CMD_16_WRITE( addr, write_val ), CMD_16_WRITE_NUM_BITS );
                delay( 25 );

                Serial.print( " ADDR: " );
                Serial.print( addr, HEX );
                Serial.print( " WRITE: " );
                Serial.print( write_val, HEX );
                Serial.print( " READ: " );
                delay( 25 );

                read_val = read_16( addr );
                delay( 25 );

                if ( read_val == write_val )
                {
                    Serial.print( read_val, HEX );
                    Serial.println( " OK" );
                    flash_result( LED_GREEN );
                }
                else
                {
                    Serial.print( read_val, HEX );
                    Serial.println( " FAIL" );
                    flash_result( LED_RED );
                }
            }
        }

        /*
            Disable writing to the EEPROM, using the 16-bit disable command.
        */
        write_cmd( ( uint16_t )CMD_16_WDS, CMD_16_WDS_NUM_BITS );

    #elif ( RUN_AS == TEST_READ_WRITE_8 )

        /*
            NOTE: to operate in 8-bit mode, the EEPROM's ORG pin must be connected
            to ground, VSS. In my testing I sometimes had a 470 ohm pull-down resistor
            between the pin and ground and that worked fine too.

            Enable writing to the EEPROM, using the 8-bit write enable command.
        */
        write_cmd( ( uint16_t )CMD_8_WEN, CMD_8_WEN_NUM_BITS );
        delay( 25 );

        while ( 1 )
        {
            uint8_t addr;
            uint8_t read_val;
            uint8_t write_val;
    
            for ( addr = 0x0; addr <= 0x7F; addr++ )
            {
                write_val = ~( uint8_t )random( 256 );
    
                write_cmd( assemble_CMD_8_WRITE( addr, write_val ), CMD_8_WRITE_NUM_BITS );
                delay( 25 );

                Serial.print( " ADDR: " );
                Serial.print( addr, HEX );
                Serial.print( " WRITE: " );
                Serial.print( write_val, HEX );
                Serial.print( " READ: " );
                delay( 25 );

                read_val = read_8( addr );
                delay( 25 );

                if ( read_val == write_val )
                {
                    Serial.print( read_val, HEX );
                    Serial.println( " OK" );
                    flash_result( LED_GREEN );
                }
                else
                {
                    Serial.print( read_val, HEX );
                    Serial.println( " FAIL" );
                    flash_result( LED_RED );
                }
            }
        }

        /*
            Disable writing to the EEPROM, using the 8-bit disable command.
        */
        write_cmd( ( uint16_t )CMD_8_WDS, CMD_8_WDS_NUM_BITS );

    #else

        /*
            Place your own custom task here, if you like...
        */
        #error "Undefined build configuration definition RUN_AS"

    #endif

    /*
        Hang here
    */
    while ( 1 )
    {
        /*
            Do nothing
        */
        ;

    }

}
