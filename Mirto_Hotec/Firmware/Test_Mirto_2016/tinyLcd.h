// tinyLcd.h
#include <U8g2lib.h> // for LCD

static char _buf[64]; // for printf

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

static U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);

//temp hack, move this into class
static const int nbrTextLines = 5;
static const int charsPerLine = 21;
static const int lcdHeight    = 64;
static const int fontHeight   = 11;  // font ascender (cap) height
static const int lineSpace    = (lcdHeight - fontHeight) / 4;
static const int txtBottom[nbrTextLines] = {fontHeight, lineSpace + fontHeight, lineSpace * 2 + fontHeight, lineSpace * 3 + fontHeight, lcdHeight - 1};
static constexpr int txtTop[nbrTextLines]  = {0, lineSpace, lineSpace * 2, lineSpace * 3, lcdHeight - fontHeight - 1};
char textBuffer[nbrTextLines][charsPerLine + 2];

class LCD
{
  public:

    LCD() {};

    void begin()
    {
      u8g2.begin();
      u8g2.setFont( u8g2_font_crox1hb_tr );
      clear();
    }

    void printf(const char* format, ...)
    {      
      va_list args;    
      u8g2.clearBuffer(); // the pixel buffer
      u8g2.sendBuffer(); 
      // scroll textBuffer down one line
      for ( int i = nbrTextLines - 1; i > 0;  i--) {
        memcpy(&textBuffer[i][0], &textBuffer[i - 1][0], charsPerLine);               
      }
      // format the top line
      va_start (args, format);
      vsprintf(_buf, format, args );
      strlcpy(&textBuffer[0][0], _buf, charsPerLine);
      show();
     
    }

    void text(char *txt, int row)
    {
      if ( strcmp(textBuffer[row], righTrimSpace(txt)) != 0) {
        //Serial.printf(" new txt on line %d: {%s}, prv txt [%s]\n", row, txt, textBuffer[row] );
        clearLine(row );
        text(txt, row, 0);
      }
    }

    char *righTrimSpace( char *txt)
    {
      char *end;
      int len;
      const char c = ' '; // the character to trim

      len = strlen(txt);
      while ( *txt && len)
      {
        end = txt + len - 1;
        if ( c == *end)
          *end = 0;
        else
          break;
        len = strlen(txt);
      }
      return (txt);
    }

    void text(char *txt, int row, int column)
    {
      strlcpy(&textBuffer[row][column], righTrimSpace(txt), sizeof(textBuffer[row]) - column);
      show();
    }

    void show()
    {
      for ( int line = 0; line < nbrTextLines; line++) {
        u8g2.setCursor(0, txtBottom[line]);
        u8g2.print(&textBuffer[line][0]);
        //Serial.printf("show: line = %d, y = %d, %s\n", line, txtBottom[line], &textBuffer[line][0]);
      }
      u8g2.sendBuffer();         // transfer internal memory to the display
    }

    void clear()
    {
      memset(textBuffer, 0, sizeof(textBuffer)); // the text buffer
      u8g2.clearBuffer(); // the pixel buffer
      u8g2.sendBuffer();
    }

    void clearLine(int line)
    {
      int width =  u8g2.getDisplayWidth();
      u8g2.setDrawColor(0);
      u8g2.drawBox( 0, txtTop[line], width, txtBottom[line]);
      u8g2.setDrawColor(1);
      u8g2.sendBuffer();
     // Serial.printf("ClearLine: x=%d, y=%d, x1=%d, y1=%d\n", 0, txtTop[line], width, txtBottom[line] );
    }

    void hGraph(int line, int value)
    {
      int width =  u8g2.getDisplayWidth() - 1;
      u8g2.setDrawColor(0);
      u8g2.drawBox( 0, txtTop[line], width - 1, fontHeight);
      u8g2.setDrawColor(1);
      u8g2.drawBox( 0, txtTop[line] + 2, map(value, 0, 100, 0, width), fontHeight - 1);
      u8g2.sendBuffer();
    }

    void hGraph(char const * title, int value1, int value2, int value3, int value4)
    {
      int width =  u8g2.getDisplayWidth() - 1;
      u8g2.drawStr(0, 12, title);
      u8g2.drawBox( 0, 16, map(value1, 0, 100, 0, width), 8);
      u8g2.drawBox( 0, 28, map(value2, 0, 100, 0, width), 8);
      u8g2.drawBox( 0, 40, map(value3, 0, 100, 0, width), 8);
      u8g2.drawBox( 0, 52, map(value4, 0, 100, 0, width), 8);  ;
    }
};

LCD lcd;
