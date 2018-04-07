///////////////////////////////////////////////////////////////////////////////////////////
// Expects Serial Rx from the C2000 on PA4

const int Led1 = PN_1;
const int Led2 = PN_0;
const int Led3 = PF_4;
const int Led4 = PF_0;

int toggle_led_1 = 0;
int toggle_led_2 = 0;
int toggle_led_3 = 0;
int toggle_led_4 = 0;

void setup() 
{
  Serial.begin(9600);
  Serial3.begin(9600);
  pinMode(Led1, OUTPUT);
  pinMode(Led2, OUTPUT);
  pinMode(Led3, OUTPUT);
  pinMode(Led4, OUTPUT);
}

///////////////////////////////////////////////////////////////////////////////
struct RingBuff
{
  uint8_t * const buff_ptr;
  const size_t    buff_size;
  
  uint32_t        push_index;
  uint32_t        pop_index;
  size_t          count;
};

void    ringPush(   struct RingBuff* Ring, uint8_t push_byte);
uint8_t ringPop(    struct RingBuff* Ring);

bool    ringIsEmpty(struct RingBuff* Ring);
bool    ringIsFull( struct RingBuff* Ring);
size_t  ringBytes(  struct RingBuff* Ring);

uint8_t my_poluloCRC(uint8_t *buffer, uint8_t my_size);

uint8_t serial_buff[256];
struct  RingBuff SerialRing = {serial_buff, (sizeof(serial_buff) + 1), 0, 0, 0};

///////////////////////////////////////////////////////////////////////////////

uint32_t loop_count = 0;

void loop() 
{ 
  loop_count = (loop_count + 1) % 120000000;
  
  if (!(loop_count % 120000))
  {
    // Led1 on loop
    toggle_led_1 = !toggle_led_1;
    digitalWrite(Led1 , toggle_led_1);
    Serial.println("empty" );   
  } else 
  {
    while (Serial3.available() > 1)
    {
      ///////////////////////////////////////////
      // Led1 toggle every 120 loop
      
      loop_count = (loop_count + 1) % 120000000;  
      if (!(loop_count % 120))
      {      
        toggle_led_1 = !toggle_led_1;
        digitalWrite(Led1 , toggle_led_1);    
      }

     ///////////////////////////////////////
      // Led2 toggle uart char available 
           
      toggle_led_2 = !toggle_led_2;
      digitalWrite(Led2, toggle_led_2);
                
      uint8_t read_char = Serial3.read();

      if (!(ringIsFull(&SerialRing)))
      {
        ringPush(&SerialRing, read_char);      
      }

      ///////////////////////////////////////////////////////////
      // Led3 toggle ring ecap uint32_t available 
         
      if(ringBytes(&SerialRing) >= 5)
      {  
        toggle_led_3 = !toggle_led_3;
        digitalWrite(Led3, toggle_led_3);

        uint8_t ecap_buff[4];

        ecap_buff[0] = ringPop(&SerialRing);
        ecap_buff[1] = ringPop(&SerialRing);
        ecap_buff[2] = ringPop(&SerialRing);
        ecap_buff[3] = ringPop(&SerialRing);

        uint32_t ecap = (ecap_buff[0] << 24);
        ecap   = ecap | (ecap_buff[1] << 16);
        ecap   = ecap | (ecap_buff[2] <<  8);
        ecap   = ecap | (ecap_buff[3]);

        uint8_t sent_crc = ringPop(&SerialRing);
        uint8_t calc_crc = my_poluloCRC(ecap_buff, 4); 
             
        if((sent_crc == calc_crc) && (sent_crc != 0))
        {
          Serial.print("SUCCESS: ");
          Serial.println(ecap, DEC);                

        } else
        {       
          Serial.print("FAIL: ");
          Serial.println(ecap, DEC);  
          uint8_t trash = ringPop(&SerialRing);
        }    
      }     
    }
  }
}

/////////////////////////////////////////////////////////////////////

bool ringIsEmpty(struct RingBuff* Ring)
{
  return (Ring->count == 0);
}

//////////////////////////////////////////////////////////////////////

bool ringIsFull(struct RingBuff* Ring)
{
  return ((Ring->push_index == Ring->pop_index) && (Ring->count != 0));
}

/////////////////////////////////////////////////

size_t ringBytes(struct RingBuff* Ring)
{
  return Ring->count;
}

/////////////////////////////////////////////////////////////////////

void ringPush(struct RingBuff* Ring, uint8_t push_byte)
{
  if(ringIsFull(Ring))
  {
    return;

  } else {

    Ring->buff_ptr[Ring->push_index] = push_byte;
    Ring->push_index = (Ring->push_index + 1) % Ring->buff_size;
    Ring->count++;
    return;
  }
}

///////////////////////////////////////////////////////////////

uint8_t ringPop(struct RingBuff* Ring)
{
  if(ringIsEmpty(Ring))
  {
    return 0;

  } else {

    uint8_t pop_byte = Ring->buff_ptr[Ring->pop_index];
    Ring->pop_index = (Ring->pop_index + 1) % Ring->buff_size;
    Ring->count--;
    return pop_byte;
  }
}


////////////////////////////////////////////////////////

//https://www.pololu.com/docs/0J44/6.7.6

uint8_t my_poluloCRC(uint8_t *buffer, uint8_t my_size)
{
  uint8_t crc7 = 0x00;
  uint8_t i    = 0x00;
  uint8_t j    = 0x00;

  for (i = 0; i < my_size; i++)
  {
      crc7 = crc7 ^ (buffer[i] & 0xFF);

    for (j = 0; j < 8; j++)
    {
      if (crc7 & 0x01)
      {
        crc7 = crc7 ^ 0x91;
      }

      crc7 = crc7 >> 1;
    }
  }
  return crc7;
}

