#include <PixyI2C.h>

class ServoLoop
{
public:
  ServoLoop(int32_t pgain, int32_t dgain);

  void update(int32_t error);

  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_pgain;
  int32_t m_dgain;
};

ServoLoop::ServoLoop(int32_t pgain, int32_t dgain)
{
  m_pos = PIXY_RCS_CENTER_POS;
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}

void ServoLoop::update(int32_t error)
{
  long int vel;
  char buf[32];
  if (m_prevError!=0x80000000)
  {
    vel = (error*m_pgain + (error - m_prevError)*m_dgain)>>10;
    //sprintf(buf, "%ld\n", vel);
    //Serial.print(buf);
    m_pos += vel;
    if (m_pos>PIXY_RCS_MAX_POS)
      m_pos = PIXY_RCS_MAX_POS;
    else if (m_pos<PIXY_RCS_MIN_POS)
      m_pos = PIXY_RCS_MIN_POS;
  }
  m_prevError = error;
}
