/*
   Firmware Jiga de Teste p/ Placa Sensor de Posição Abs.

   Versão 2.0
   Desenvolvido em 02/2022

*/

#include <BspJigaSensorAbs.h>
#include <arduino-timer.h>

//#include "BspJigaSensorAbs.h"

#define DELAY_INIT         1000
#define DELAY_SET_REFER    2000
#define DELAY_SET_CORRENTE   200

// NA MEDIÇÃO DE 3V3 AS PLACAS MEDIRAM EM TORNO DE 1922 ( 1,54V -> 3,097V)
#define MIN_3V3       1886 //     1948     // VALOR REAL: 3,2V    // Valor mínimo da medição 3V3. Div. de tensão de "3,2"V para 1,55V e considerando R4 e R5 com 5% de tolerancia, pior caso (R5 9,5K e R4 10,5k). Tensão mínima 1.52 V (1886)
#define MAX_3V3       2084 //     2147         // Valor máximo da medição 3V3. Div. de tensão de "3,2V" para 1,68V e considerando R4 e R5 com 5% de tolerancia, pior caso (R5 10,5K e R4 9,5k). Tensão máxima 1.68 V  (2084)

#define CORREN_MIN_100  2350    //        3411 // 2400 //2830         // Valor mínimo aceitável na corrente de 100% (20mA). Considerando R15 com -5%, pior caso (114 ohms). Tensão mínima 2,28 V (2830). 
#define CORREN_MAX_100  3127    //        3811 // 3127         // Valor máximo aceitável na corrente de 100% (20mA). Considerando R15 com +5%, pior caso (126 ohms). Tensão máxima 2,52 V (3127).

#define CORREN_MIN_50   1580    //       2888 // 1600 //1697         // Valor mínimo aceitável na corrente de 50% (12mA). Considerando R15 com -5%, pior caso (114 ohms). Tensão mínima 1,368 V (1697). 
#define CORREN_MAX_50   1876    //       3288 //1876         // Valor máximo aceitável na corrente de 50% (12mA). Considerando R15 com +5%, pior caso (126 ohms). Tensão máxima 1,512 V (1876).

#define CORREN_MIN_0    560     //       2189 //565          // Valor mínimo aceitável na corrente de 0% (4mA). Considerando R15 com -5%, pior caso (114 ohms). Tensão mínima 0,456 V (565). 
#define CORREN_MAX_0    625     //       2589 // 625          // Valor máximo aceitável na corrente de 0% (4mA). Considerando R15 com +5%, pior caso (126 ohms). Tensão máxima 0,504 V (625).

#define KEY_CURR_CTL 0x5566

#define SLAVE_ADDRESS      0x26
#define I2C_SUCESS         0
#define I2C_NACK           2

#define STR_INFO           "POS. ABS"

#define SENSOR_1_VAL_NOM     262         // Valor nominal 265
#define SENSOR_2_VAL_NOM     271         // Valor nominal 270
#define SENSOR_3_VAL_NOM     278         // Valor nominal 260
#define SENSOR_4_VAL_NOM     266         // Valor nominal 265
#define SENSOR_5_VAL_NOM     261         // Valor nominal 258

#define TOLER_SENSORES     5

#define SENSOR_OK          0

#define POSI_STATUS_OK     0x80         // DEFINIR ----- STATUS ESPERADO NA LEITURA DA POSIÇÃO
#define POSI_VAL_MIN       0x00         // DEFINIR ----- VALOR MINIMO ACEITAVEL DA LEITURA DA POSIÇÃO
#define POSI_VAL_MAX       0xFF         // DEFINIR ----- VALOR MAXIMO ACEITAVEL DA LEITURA DA POSIÇÃO

#define SET_CORREN_100     1000         // 26.873.856
#define SET_CORREN_50      500
#define SET_CORREN_0       0


hw_timer_t * timer = NULL;


auto timer_resset = timer_create_default();
Timer<> default_timer;


//Timer<16, millis, const char *> timer_resset;

bool jiga_was_resset = false;
bool timer_enab = false;
uint16_t valCur;
uint8_t buf [5];
uint8_t cont_retent, cont_med_retent, cont_pos_retent;
uint8_t sum;


//Valor de status e posição de cada sensor
typedef union
{
  struct st_read_sensors
  {
    uint8_t S1_stat;
    uint16_t S1_val;
    uint8_t S2_stat;
    uint16_t S2_val;
    uint8_t S3_stat;
    uint16_t S3_val;
    uint8_t S4_stat;
    uint16_t S4_val;
    uint8_t S5_stat;
    uint16_t S5_val;
  } st_sens;
  uint8_t bytesSensores[sizeof(st_sens)];
} u_read_sensors;


//Valor da posição
typedef union
{
  struct st_val_pos
  {
    uint32_t posit;
    uint8_t stat;
    uint8_t cs;
  } st_vpos;
  uint8_t bytesRec[sizeof(st_vpos)];
} u_val_pos;


enum en_byte_write {
  // Read
  IFR_POS = 0,
  IFR_INFO,
  IFR_DIAG,
  // Write
  IFR_CMD_ZERO = 0x80,
  IFR_CMD_ZERO_VAL,
  IFR_CMD_CURR,
  IFR_CMD_CURR_VAL,
} en_byte_wr;


// Status Geral de Testes
enum en_status_teste {
  STT_RUN,
  STT_FAIL,
  STT_FINISH
} stt_geral;


// Status de Teste
typedef enum
{
  STT_TEST_INIT = 0,
  STT_TEST_3V3,
  STT_TEST_I2C_CANAL,
  STT_TEST_COMUNIC_SENSORES,
  STT_TEST_LEITURA_POSICAO,
  STT_TEST_CORRENTE,
  STT_TEST_FIM
  
} STT_TEST_t;


// Enumeração das Falhas
enum en_fail {
  FAIL_3V3 =0,
  
  FAIL_I2C_COMUN,
  
  FAIL_SENSOR_1,
  FAIL_SENSOR_2,
  FAIL_SENSOR_3,
  FAIL_SENSOR_4,
  FAIL_SENSOR_5,
  
  FAIL_I2C_POSICAO,
  FAIL_I2C_CHECKSUM,

  FAIL_MED_CUR_0,
  FAIL_MED_CUR_50,
  FAIL_MED_CUR_100,
  
  FAIL_NONE
} en_test_fail;


// Status de medição de corrente
enum en_medicao_curr
{
  STT_MEDICAO_INIT,
  STT_MEDICAO_ENVIA_CMD_CURR,
  STT_MEDICAO_ENVIA_BUF,
  STT_MEDICAO_RETENT_CMD,
  STT_MEDICAO_WAIT_ESTAB,
  STT_MEDICAO_GET_VAL,
  STT_MEDICAO_VERIF_100,
  STT_MEDICAO_VERIF_50,
  STT_MEDICAO_VERIF_0,
  STT_MEDICAO_RETENT,
  STT_MEDICAO_FALHA
} stt_medicao_curr;


// Status geral do teste de corrente
enum en_teste_curr_geral
{
  TEST_CURR_DISAB_5V,
  TEST_CURR_WAIT_RESSET,
  TEST_CURR_WAIT_INIT,
  TEST_CURR_MED_100,
  TEST_CURR_MED_50,
  TEST_CURR_MED_0,
  TEST_CURR_FALHA
} stt_test_curr;


enum en_read_posic
{
  STT_READ_POS_SET_REFER,
  STT_READ_POS_SET_REINIT_TIME,
  STT_READ_POS_WAIT_REINIT_TIME,
  STT_READ_POS_TEST_REINIT,
  STT_READ_POS_CHECK_SUM,
  STT_READ_POS_VERIF_POSICAO,
  STT_READ_POS_FALHA

} stt_read_pos;



STT_TEST_t stt_test;
u_val_pos u_pos;
u_read_sensors u_sensors;
BspJigSensAbs bsp = BspJigSensAbs();



uint8_t BUF_SET_REFER[] =  {0x81, 0xAA, 0xBB, 0xCC, 0xDD};


// Testa a comunicação I2C.
bool
jiga_pos_abs_i2c_test()
{
  char chInfo[8];
  if ( bsp.bsp_i2c_write( IFR_INFO ) == I2C_SUCESS )
  {
    bsp.bsp_i2c_read( chInfo, sizeof(chInfo) );
    String strInfo( chInfo );
    if (strInfo == STR_INFO)
    {
      return true;
    }
  }
  en_test_fail = FAIL_I2C_COMUN;
  return false;
}


// Medição do sinal 3V3. Uma única leitura.
bool
jiga_pos_abs_test_3v3()
{
  uint16_t val = bsp.bsp_get_3V3();

  if ((val >= MIN_3V3) && (val <= MAX_3V3))
  {
    return true;
  }
  else
  {
    en_test_fail = FAIL_3V3;
    return false;
  }
}

void
jiga_pos_abs_sinal_init()            // Sinalização de inicialização: Pisca os dois leds 3 vezes
{
  for (int i = 0; i < 5; i++) {
    bsp.bsp_set_led_status_green();
    bsp.bsp_set_led_status_red();
    delay(500);
    bsp.bsp_clear_led_status_green();
    bsp.bsp_clear_led_status_red();
    delay(500);
  }
}

void IRAM_ATTR blink_LED()
{
  bsp.bsp_toggle_led_status_green();
}


// Rotina de inicialização
void
jiga_pos_abs_init()
{
  bsp.bsp_enab_5V_ext();
  bsp.bsp_i2c_set_slave_addres( SLAVE_ADDRESS );
  delay(DELAY_INIT);
  bsp.bsp_i2c_start();
  jiga_pos_abs_sinal_init();                          // sinaliza pelos LEDs a inicialização

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &blink_LED, true);
  timerAlarmWrite(timer, 300000, true);
  timerAlarmEnable(timer);

  stt_test_curr = TEST_CURR_DISAB_5V;
  stt_medicao_curr = STT_MEDICAO_INIT;
  
  stt_read_pos = STT_READ_POS_SET_REFER;
  cont_pos_retent =0;
}


bool
jiga_pos_abs_i2c_test_sensors()       // Lê cada um dos sensores
{
  uint8_t sumS = 0;

  if ( bsp.bsp_i2c_write( IFR_DIAG ) == I2C_SUCESS )
  {
    bsp.bsp_i2c_read(u_sensors.bytesSensores, sizeof(u_sensors.bytesSensores));

    if( ! ((u_sensors.st_sens.S1_stat == SENSOR_OK) && (u_sensors.st_sens.S1_val >= (SENSOR_1_VAL_NOM - TOLER_SENSORES)) && (u_sensors.st_sens.S1_val <= (SENSOR_1_VAL_NOM + TOLER_SENSORES))))
    {
      en_test_fail = FAIL_SENSOR_1;
      Serial.println( "FAIL_SENSOR_1" );
      return false;
    }

    if ( ! ((u_sensors.st_sens.S2_stat == SENSOR_OK) && (u_sensors.st_sens.S2_val >= (SENSOR_2_VAL_NOM - TOLER_SENSORES)) && (u_sensors.st_sens.S2_val <= (SENSOR_2_VAL_NOM + TOLER_SENSORES))))
    {
      en_test_fail = FAIL_SENSOR_2;
      return false;
    }

    if( ! ((u_sensors.st_sens.S3_stat == SENSOR_OK) && (u_sensors.st_sens.S3_val >= (SENSOR_3_VAL_NOM - TOLER_SENSORES)) && (u_sensors.st_sens.S3_val <= (SENSOR_3_VAL_NOM + TOLER_SENSORES))))
    {
      en_test_fail = FAIL_SENSOR_3;
      return false;
    }

    if( ! ((u_sensors.st_sens.S4_stat == SENSOR_OK) && (u_sensors.st_sens.S4_val >= (SENSOR_4_VAL_NOM - TOLER_SENSORES)) && (u_sensors.st_sens.S4_val <= (SENSOR_4_VAL_NOM + TOLER_SENSORES)))) {
    //if(  ((u_sensors.st_sens.S4_stat == SENSOR_OK) && (u_sensors.st_sens.S4_val >= (SENSOR_4_VAL_NOM - TOLER_SENSORES)) && (u_sensors.st_sens.S4_val <= (SENSOR_4_VAL_NOM + TOLER_SENSORES)))) {
      en_test_fail = FAIL_SENSOR_4;
      return false;
    }

    if( ! (u_sensors.st_sens.S5_stat == SENSOR_OK) && (u_sensors.st_sens.S5_val >= (SENSOR_5_VAL_NOM - TOLER_SENSORES)) && (u_sensors.st_sens.S5_val <= (SENSOR_5_VAL_NOM + TOLER_SENSORES))){
      en_test_fail = FAIL_SENSOR_5;
      return false;
    }
    return true;

  }
  en_test_fail = FAIL_I2C_COMUN;
  return false;
}


bool
jiga_pos_abs_i2c_set_refer()          // Seta a referencia.
{
  if (!bsp.bsp_i2c_write( IFR_CMD_ZERO ))
  {
    if (!bsp.bsp_i2c_write( BUF_SET_REFER, 5 ))
    {
      return true;
    }
  }
  en_test_fail = FAIL_I2C_COMUN;
  return false;
}


bool
jiga_pos_abs_i2c_read_position()      // Lê a posição
{
  switch ( stt_read_pos )
  {
    case STT_READ_POS_SET_REFER:
      if ( jiga_pos_abs_i2c_set_refer() )
      {
        stt_read_pos = STT_READ_POS_SET_REINIT_TIME;
        Serial.println(" STT_READ_POS_SET_REFER OK");
      }
      else
      {
        Serial.println(" STT_READ_POS_SET_REFER FAIL");
        en_test_fail = FAIL_I2C_COMUN;
        stt_read_pos = STT_READ_POS_FALHA;
      }
      break;


    case STT_READ_POS_SET_REINIT_TIME:
      jiga_was_resset = false;
      timer_resset.in( 2000, jiga_pos_abs_shutdown_time );
      timer_enab = true;
      stt_read_pos = STT_READ_POS_WAIT_REINIT_TIME;
      break;


    case STT_READ_POS_WAIT_REINIT_TIME:
      if ( jiga_was_resset == true )
      {
        stt_read_pos = STT_READ_POS_TEST_REINIT;
        cont_retent = 0;
      }
      break;


    case STT_READ_POS_TEST_REINIT:
      if( bsp.bsp_i2c_write( IFR_POS ) == I2C_SUCESS )
      {
        bsp.bsp_i2c_read( u_pos.bytesRec, sizeof(u_pos.bytesRec) );
        stt_read_pos = STT_READ_POS_CHECK_SUM;
        cont_retent=0;
      }
      else
      {
        cont_retent++;
        delay( 10 );
        if ( cont_retent > 3 )
        {
          stt_read_pos = STT_READ_POS_FALHA;
          en_test_fail = FAIL_I2C_COMUN;
        }
      }
      break;


    case STT_READ_POS_CHECK_SUM:
      sum = 0;
      for (int i = 0; i < 5; i++)
      {
        sum += u_pos.bytesRec[i];
      }

      if (u_pos.st_vpos.cs == sum)
      {
        stt_read_pos = STT_READ_POS_VERIF_POSICAO;
      }
      else
      {
        en_test_fail = FAIL_I2C_CHECKSUM;
        stt_read_pos = STT_READ_POS_FALHA;
      }
      break;


    case STT_READ_POS_VERIF_POSICAO:
      if( (u_pos.st_vpos.stat == POSI_STATUS_OK) && (u_pos.st_vpos.posit >= POSI_VAL_MIN) && (u_pos.st_vpos.posit <= POSI_VAL_MAX) )
      {
        return true;
      }
      else
      {
        cont_pos_retent++;
        if( cont_pos_retent < 5 )
        {
          Serial.println(" RETENT READ POS ");
          stt_read_pos = STT_READ_POS_SET_REFER;
          delay(500);
        }
        else
        {
          en_test_fail = FAIL_I2C_POSICAO;
          stt_read_pos = STT_READ_POS_FALHA; 
        }
      }
      break;


    case STT_READ_POS_FALHA:
      break;
  }
  return false;
}




bool jiga_pos_abs_shutdown_time( void *)
{
  jiga_was_resset = true;
  return false;   // Não repetir o timer
}



// Medição da corrente. Uma leitura para cada valor de corrente (100%, 50% e 0%)
bool
jiga_pos_abs_test_corrente()
{
  switch ( stt_test_curr )
  {
    // Desabilita a alimentação da placa sob teste
    case TEST_CURR_DISAB_5V:
      bsp.bsp_disab_5V_ext();
      jiga_was_resset = false;
      timer_resset.in( 3000, jiga_pos_abs_shutdown_time );
      stt_test_curr = TEST_CURR_WAIT_RESSET;
      break;


    // Aguarda 3 segundos com a placa sob teste desenergizada
    case TEST_CURR_WAIT_RESSET:
      if ( jiga_was_resset == true )
      {
        bsp.bsp_enab_5V_ext();
        jiga_was_resset = false;
        timer_resset.in( DELAY_INIT, jiga_pos_abs_shutdown_time );
        stt_test_curr = TEST_CURR_WAIT_INIT;
      }
      break;


    // Aguarda 1 segundo após a reinicialização
    case TEST_CURR_WAIT_INIT:
      if ( jiga_was_resset == true )
      {
        jiga_was_resset = false;
        stt_test_curr = TEST_CURR_MED_100;
      }
      break;


    // Teste de corente em 100%
    case TEST_CURR_MED_100:
      if ( jiga_pos_abs_med_curr( SET_CORREN_100 ))
      {
        stt_test_curr = TEST_CURR_MED_50;
      }
      else
      {
        if ( stt_medicao_curr == STT_MEDICAO_FALHA )
        {
          stt_test_curr = TEST_CURR_FALHA;
        }
      }
      break;


    // Teste de corente em 50%
    case TEST_CURR_MED_50:
      if ( jiga_pos_abs_med_curr( SET_CORREN_50 ))
      {
        stt_test_curr = TEST_CURR_MED_0;
      }
      else
      {
        if ( stt_medicao_curr == STT_MEDICAO_FALHA )
        {
          stt_test_curr = TEST_CURR_FALHA;
        }
      }
      break;


    // Teste de corente em 0%
    case TEST_CURR_MED_0:
      if ( jiga_pos_abs_med_curr( SET_CORREN_0 ))
      {
        return true;
      }
      else
      {
        if ( stt_medicao_curr == STT_MEDICAO_FALHA )
        {
          stt_test_curr = TEST_CURR_FALHA;
        }
      }
      break;


    case TEST_CURR_FALHA:
      break;
  }
  return false;
}


bool
jiga_pos_abs_med_curr( uint16_t percCorrente )
{
  switch ( stt_medicao_curr )
  {
    case STT_MEDICAO_INIT:
      buf[0] = IFR_CMD_CURR_VAL;
      buf[1] = KEY_CURR_CTL;
      buf[2] = KEY_CURR_CTL >> 8;
      buf[3] = percCorrente;
      buf[4] = percCorrente >> 8;
      stt_medicao_curr = STT_MEDICAO_ENVIA_CMD_CURR;
      cont_retent = 0;
      cont_med_retent = 0;


    case STT_MEDICAO_ENVIA_CMD_CURR:
      if( bsp.bsp_i2c_write( IFR_CMD_CURR ) == I2C_SUCESS )
      {
        stt_medicao_curr = STT_MEDICAO_ENVIA_BUF;
      }
      else
      {
        stt_medicao_curr = STT_MEDICAO_RETENT_CMD;
      }
      break;


    case STT_MEDICAO_ENVIA_BUF:
      if ( bsp.bsp_i2c_write( buf, 5 ) == I2C_SUCESS )
      {
        stt_medicao_curr = STT_MEDICAO_WAIT_ESTAB;
        timer_resset.in( 50, jiga_pos_abs_shutdown_time );
      }
      else
      {
        stt_medicao_curr = STT_MEDICAO_RETENT_CMD;
      }
      break;


    case STT_MEDICAO_RETENT_CMD:
      cont_retent++;
      Serial.println("RETENT");
      if( cont_retent > 8 )
      {
        en_test_fail = FAIL_I2C_COMUN;
        stt_medicao_curr = STT_MEDICAO_FALHA;
      }
      else
      {
        stt_medicao_curr = STT_MEDICAO_ENVIA_CMD_CURR;
        delay( 10 );
      }
      break;


    case STT_MEDICAO_WAIT_ESTAB:
      if ( jiga_was_resset == true )
      {
        jiga_was_resset = false;
        stt_medicao_curr = STT_MEDICAO_GET_VAL;
      }
      break;

    
    case STT_MEDICAO_GET_VAL:
      Serial.println("STT_MEDICAO_GET_VAL");
      valCur = bsp.bsp_get_corrente();
      if( percCorrente == SET_CORREN_100 )
      {
        Serial.println("SET_CORREN_100");
        Serial.println(valCur);
        stt_medicao_curr = STT_MEDICAO_VERIF_100;
      } 
      else if( percCorrente == SET_CORREN_50 )
      {
        Serial.println("SET_CORREN_50");
        Serial.println(valCur);
        stt_medicao_curr = STT_MEDICAO_VERIF_50;
      }
      else if( percCorrente == SET_CORREN_0 )
      {
        Serial.println("SET_CORREN_0");
        Serial.println(valCur);
        stt_medicao_curr = STT_MEDICAO_VERIF_0;
      }
      else
      {
         stt_medicao_curr = STT_MEDICAO_FALHA;
      }
      break;


    case STT_MEDICAO_VERIF_100:
      if( ( valCur >= CORREN_MIN_100 ) && ( valCur <= CORREN_MAX_100 ) )
      {
        stt_medicao_curr = STT_MEDICAO_INIT;
        en_test_fail = FAIL_NONE;
        return true;
      }
      else
      {
        stt_medicao_curr = STT_MEDICAO_RETENT;
        en_test_fail = FAIL_MED_CUR_100;
      }
      break;


    case STT_MEDICAO_VERIF_50:
      if( ( valCur >= CORREN_MIN_50 ) && ( valCur <= CORREN_MAX_50 ) )
      {
        stt_medicao_curr = STT_MEDICAO_INIT;
        en_test_fail = FAIL_NONE;
        return true;
      }
      else
      {
        stt_medicao_curr = STT_MEDICAO_RETENT;
        en_test_fail = FAIL_MED_CUR_50;
      }
      break;


    case STT_MEDICAO_VERIF_0:
      if( ( valCur >= CORREN_MIN_0 ) && ( valCur <= CORREN_MAX_0 ) )
      {
        stt_medicao_curr = STT_MEDICAO_INIT;
        en_test_fail = FAIL_NONE;
        return true;
      }
      else
      {
        stt_medicao_curr = STT_MEDICAO_RETENT;
        en_test_fail = FAIL_MED_CUR_0;
      }
      break;

    
    case STT_MEDICAO_RETENT:
      cont_med_retent++;
      if( cont_med_retent > 5 )
      {
        stt_medicao_curr = STT_MEDICAO_FALHA;
      }
      else
      {
        stt_medicao_curr = STT_MEDICAO_GET_VAL;
        delay( 10 );
      }
      break;

    case STT_MEDICAO_FALHA:
      break;
  }
  return false;
}


bool
jiga_pos_abs_run_test()
{
  switch ( stt_test )
  {
    case STT_TEST_INIT:
      jiga_pos_abs_init();
      stt_test = STT_TEST_3V3;
      break;


    case STT_TEST_3V3:
      if ( jiga_pos_abs_test_3v3() )
      {
        Serial.println("3V3 OK");
        stt_test = STT_TEST_I2C_CANAL;
        jiga_pos_abs_sinal_test_etapa_ok();
      }
      else
      {
        stt_geral = STT_FAIL;
      }
      break;


    case STT_TEST_I2C_CANAL:
      if ( jiga_pos_abs_i2c_test() )
      {
        Serial.println("CANAL I2C OK");
        stt_test = STT_TEST_COMUNIC_SENSORES;
        jiga_pos_abs_sinal_test_etapa_ok();
      }
      else
      {
        stt_geral = STT_FAIL;
      }
      break;


    case STT_TEST_COMUNIC_SENSORES:
      if ( jiga_pos_abs_i2c_test_sensors() )
      {
        Serial.println("STT_TEST_COMUNIC_SENSORES OK");
        stt_test = STT_TEST_LEITURA_POSICAO;
        jiga_pos_abs_sinal_test_etapa_ok();
      }
      else
      {
        stt_geral = STT_FAIL;
      }
      break;


    case STT_TEST_LEITURA_POSICAO:
      if ( jiga_pos_abs_i2c_read_position() )
      {
        Serial.println("STT_TEST_LEITURA_POSICAO OK");
        stt_test = STT_TEST_CORRENTE;
        jiga_pos_abs_sinal_test_etapa_ok();
      }
      else
      {
        if( stt_read_pos == STT_READ_POS_FALHA )
        {
          stt_geral = STT_FAIL;          
        }
      }
      break;


    case STT_TEST_CORRENTE:
      if ( jiga_pos_abs_test_corrente() )
      {
        Serial.println(" STT_TEST_CORRENTE OK");
        stt_geral = STT_FINISH;
        jiga_pos_abs_sinal_test_etapa_ok();
      }
      else
      {
        if ( stt_test_curr == TEST_CURR_FALHA )
        {
          stt_geral = STT_FAIL;
        }
      }
      break;
  }

}

void
jiga_pos_abs_sinal_test_etapa_fail( uint8_t num_teste )   //Sinalização do resultado de falha da etapa do teste
{
  timerAlarmDisable(timer);
  uint8_t fatorFalha;

  bsp.bsp_clear_led_status_green();
  bsp.bsp_clear_led_status_red();
  
  // Piscadas LED vermelho
  for( int i=0; i< num_teste; i++ )
  {
    bsp.bsp_set_led_status_red();
    delay( 750 );
    bsp.bsp_clear_led_status_red();
    delay( 750 );
  }
  delay( 2000 );
  
  // Adequação da enumeração de falha para o numero de piscadas do LED verde
  if( (num_teste == STT_TEST_COMUNIC_SENSORES) || ( en_test_fail == FAIL_I2C_COMUN ))
  {
    fatorFalha = 0;
  }
  else if( num_teste == STT_TEST_LEITURA_POSICAO )
  {
    fatorFalha = 5;
  }
  else if( num_teste ==  STT_TEST_CORRENTE )
  {
    fatorFalha = 7;
  }

  // Piscadas LED verde
  for( int i=0; i < (en_test_fail - fatorFalha); i++ )
  {
    bsp.bsp_set_led_status_green();
    delay( 750 );
    bsp.bsp_clear_led_status_green();
    delay( 750 );
  }
  delay( 2000 ); 
}

//Sinalização do resultado aprovado ao final do teste
void
jiga_pos_abs_sinal_end_test_ok()
{
  timerAlarmDisable(timer);
  bsp.bsp_clear_led_status_red();
  bsp.bsp_set_led_status_green();
}


//Sinalização do resultado ok da etapa do teste
void 
jiga_pos_abs_sinal_test_etapa_ok()   
{
  timerAlarmDisable(timer);
  uint8_t conPisc =0;

  bsp.bsp_clear_led_status_red();
  bsp.bsp_set_led_status_green();
  //delay(3000);
  do{
    bsp.bsp_clear_led_status_green();
    delay(100);
    bsp.bsp_set_led_status_green();
    delay(100);
    conPisc++;
  }while(conPisc < 5);
  
  timerAlarmEnable(timer);
  //delay(3000);
}



void setup()
{
  delay(2000);
  Serial.begin(9600);

  stt_geral = STT_RUN;
  Serial.println("INICIO");
}


void loop()
{
  Serial.println("INICIO");
  if ( timer_enab == true )
  {
    timer_resset.tick();
  }

  switch (stt_geral)
  {
    case STT_RUN:
      jiga_pos_abs_run_test();
      break;

    case STT_FAIL:
      Serial.println( stt_test );
      Serial.println( en_test_fail );
      jiga_pos_abs_sinal_test_etapa_fail( stt_test );
      break;

    case STT_FINISH:
      jiga_pos_abs_sinal_end_test_ok();
      break;
  }
}
