#include "app_sys.h"

#include <stdarg.h>
#include "app_port.h"

static const unsigned short ztvm_crctab16[] =
{
    0X0000, 0X1189, 0X2312, 0X329B, 0X4624, 0X57AD, 0X6536, 0X74BF,
    0X8C48, 0X9DC1, 0XAF5A, 0XBED3, 0XCA6C, 0XDBE5, 0XE97E, 0XF8F7,
    0X1081, 0X0108, 0X3393, 0X221A, 0X56A5, 0X472C, 0X75B7, 0X643E,
    0X9CC9, 0X8D40, 0XBFDB, 0XAE52, 0XDAED, 0XCB64, 0XF9FF, 0XE876,
    0X2102, 0X308B, 0X0210, 0X1399, 0X6726, 0X76AF, 0X4434, 0X55BD,
    0XAD4A, 0XBCC3, 0X8E58, 0X9FD1, 0XEB6E, 0XFAE7, 0XC87C, 0XD9F5,
    0X3183, 0X200A, 0X1291, 0X0318, 0X77A7, 0X662E, 0X54B5, 0X453C,
    0XBDCB, 0XAC42, 0X9ED9, 0X8F50, 0XFBEF, 0XEA66, 0XD8FD, 0XC974,
    0X4204, 0X538D, 0X6116, 0X709F, 0X0420, 0X15A9, 0X2732, 0X36BB,
    0XCE4C, 0XDFC5, 0XED5E, 0XFCD7, 0X8868, 0X99E1, 0XAB7A, 0XBAF3,
    0X5285, 0X430C, 0X7197, 0X601E, 0X14A1, 0X0528, 0X37B3, 0X263A,
    0XDECD, 0XCF44, 0XFDDF, 0XEC56, 0X98E9, 0X8960, 0XBBFB, 0XAA72,
    0X6306, 0X728F, 0X4014, 0X519D, 0X2522, 0X34AB, 0X0630, 0X17B9,
    0XEF4E, 0XFEC7, 0XCC5C, 0XDDD5, 0XA96A, 0XB8E3, 0X8A78, 0X9BF1,
    0X7387, 0X620E, 0X5095, 0X411C, 0X35A3, 0X242A, 0X16B1, 0X0738,
    0XFFCF, 0XEE46, 0XDCDD, 0XCD54, 0XB9EB, 0XA862, 0X9AF9, 0X8B70,
    0X8408, 0X9581, 0XA71A, 0XB693, 0XC22C, 0XD3A5, 0XE13E, 0XF0B7,
    0X0840, 0X19C9, 0X2B52, 0X3ADB, 0X4E64, 0X5FED, 0X6D76, 0X7CFF,
    0X9489, 0X8500, 0XB79B, 0XA612, 0XD2AD, 0XC324, 0XF1BF, 0XE036,
    0X18C1, 0X0948, 0X3BD3, 0X2A5A, 0X5EE5, 0X4F6C, 0X7DF7, 0X6C7E,
    0XA50A, 0XB483, 0X8618, 0X9791, 0XE32E, 0XF2A7, 0XC03C, 0XD1B5,
    0X2942, 0X38CB, 0X0A50, 0X1BD9, 0X6F66, 0X7EEF, 0X4C74, 0X5DFD,
    0XB58B, 0XA402, 0X9699, 0X8710, 0XF3AF, 0XE226, 0XD0BD, 0XC134,
    0X39C3, 0X284A, 0X1AD1, 0X0B58, 0X7FE7, 0X6E6E, 0X5CF5, 0X4D7C,
    0XC60C, 0XD785, 0XE51E, 0XF497, 0X8028, 0X91A1, 0XA33A, 0XB2B3,
    0X4A44, 0X5BCD, 0X6956, 0X78DF, 0X0C60, 0X1DE9, 0X2F72, 0X3EFB,
    0XD68D, 0XC704, 0XF59F, 0XE416, 0X90A9, 0X8120, 0XB3BB, 0XA232,
    0X5AC5, 0X4B4C, 0X79D7, 0X685E, 0X1CE1, 0X0D68, 0X3FF3, 0X2E7A,
    0XE70E, 0XF687, 0XC41C, 0XD595, 0XA12A, 0XB0A3, 0X8238, 0X93B1,
    0X6B46, 0X7ACF, 0X4854, 0X59DD, 0X2D62, 0X3CEB, 0X0E70, 0X1FF9,
    0XF78F, 0XE606, 0XD49D, 0XC514, 0XB1AB, 0XA022, 0X92B9, 0X8330,
    0X7BC7, 0X6A4E, 0X58D5, 0X495C, 0X3DE3, 0X2C6A, 0X1EF1, 0X0F78,
};


SystemInfoTypedef sysinfo;
static char LOGDEBUG[256];


uint16_t GetCrc16(const char *pData, int nLength)
{
    uint16_t fcs = 0xffff;
    while (nLength > 0)
    {
        fcs = (fcs >> 8) ^ ztvm_crctab16[(fcs ^ (*pData)) & 0xff];
        nLength--;
        pData++;
    }
    return ~fcs;

}

void LogMessage(uint8_t level, char *debug)
{
    uint16 year = 0;
    uint8  month = 0, date = 0, hour = 0, minute = 0, second = 0;
    char timedebug[20];

    if (sysinfo.logLevel < level)
        return;

    portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
    sprintf(timedebug, "[%02d:%02d:%02d] ", hour, minute, second);
    portUartSend(&usart2_ctl, (uint8_t *)timedebug, strlen(timedebug));
    portUartSend(&usart2_ctl, (uint8_t *)debug, strlen(debug));
    portUartSend(&usart2_ctl, (uint8_t *)"\r\n", 2);
}

void LogMessageWL(uint8_t level, char *buf, uint16_t len)
{
    uint16 year = 0;
    uint8  month = 0, date = 0, hour = 0, minute = 0, second = 0;
    char timedebug[20];
    if (sysinfo.logLevel < level)
        return;

    portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
    sprintf(timedebug, "[%02d:%02d:%02d] ", hour, minute, second);
    portUartSend(&usart2_ctl, (uint8 *)timedebug, tmos_strlen(timedebug));
    portUartSend(&usart2_ctl, (uint8 *)buf, len);
    portUartSend(&usart2_ctl, (uint8 *)"\r\n", 2);
}


void LogPrintf(uint8_t level, const char *debug, ...)
{
    va_list args;
    if (sysinfo.logLevel < level)
        return ;
    va_start(args, debug);
    vsnprintf(LOGDEBUG, 256, debug, args);
    va_end(args);
    LOGDEBUG[255] = 0;
    LogMessageWL(level, LOGDEBUG, tmos_strlen(LOGDEBUG));
}


void Log(uint8_t level, const char *debug, ...)
{
    va_list args;
    if (sysinfo.logLevel < level)
        return ;
    va_start(args, debug);
    vsnprintf(LOGDEBUG, 256, debug, args);
    va_end(args);
    LOGDEBUG[255] = 0;
    portUartSend(&usart2_ctl, (uint8 *)LOGDEBUG, tmos_strlen(LOGDEBUG));
}


void LogWL(uint8_t level, uint8 *buf, uint16_t len)
{
    va_list args;
    if (sysinfo.logLevel < level)
        return ;
    portUartSend(&usart2_ctl, buf, len);
}
/*------------------------------------------------------*/
//cmd1��ȫ����cmd2
uint8_t mycmdPatch(uint8_t *cmd1, uint8_t *cmd2)
{
    uint8_t ilen1, ilen2;
    if (cmd1 == NULL || cmd2 == NULL)
        return 0;
    ilen1 = strlen((char *)cmd1);
    ilen2 = strlen((char *)cmd2);
    if (ilen1 != ilen2)
        return 0;
    for (ilen1 = 0; ilen1 < ilen2; ilen1++)
    {
        if (cmd1[ilen1] != cmd2[ilen1])
            return 0;
    }
    return 1;
}
/*------------------------------------------------------*/
//��ȡ�ַ�ch��str�е�λ��
int getCharIndex(uint8_t *src, int src_len, char ch)
{
    int i;
    for (i = 0; i < src_len; i++)
    {
        if (src[i] == ch)
            return i;
    }
    return -1;
}
/*------------------------------------------------------*/
//ƥ��str2�Ƿ���ȫ��ͬ��str1��ͷ��
int my_strpach(char *str1, const char *str2)
{
    int i = 0, len;
    if (str1 == NULL || str2 == NULL)
        return 0;
    len = tmos_strlen((char *)str2);
    for (i = 0; i < len; i++)
    {
        if (str1[i] != str2[i])
            return 0;
    }
    return 1;
}
/*------------------------------------------------------*/
//��ȡstr2��str1�е�λ��
int my_getstrindex(char *str1, const char *str2, int len)
{
    uint16_t strsize;
    uint16_t i = 0;
    if (str1 == NULL || str2 == NULL || len <= 0)
        return -1;
    strsize = strlen(str2);
    if (len < strsize)
        return -2;
    for (i = 0; i <= (len - strsize); i++)
    {
        if (str1[i] == str2[0])
        {
            if (my_strpach(&str1[i], str2))
            {
                return i;
            }
        }
    }
    return -3;
}
/*------------------------------------------------------*/
//�ж�str2�Ƿ�Ϊstr1���ִ�
//str1
//str2 ƥ������
//len  str1�Ĵ�����
int my_strstr(char *str1, const char *str2, int len)
{
    int strsize;
    int i = 0;
    strsize = strlen(str2);
    for (i = 0; i <= (len - strsize); i++)
    {
        if (str1[i] == str2[0])
        {
            if (my_strpach(&str1[i], str2))
            {
                return 1;
            }
        }
    }
    return 0;
}
/*------------------------------------------------------*/
//ʶ��OK
int distinguishOK(char *buf)
{
    if (strstr(buf, "OK") != NULL)
        return 1;
    return 0;
}
/*------------------------------------------------------*/
//��ȡ�ַ�ch������str�е�n��λ�õ�����
int16_t getCharIndexWithNum(uint8_t *src, uint16_t src_len, uint8_t ch, uint8_t num)
{
    int i, count = 0;
    if (src == NULL)
        return -1;
    for (i = 0; i < src_len; i++)
    {
        if (src[i] == ch)
        {
            ++count;
            if (count == num)
            {
                return i;
            }
        }
    }
    return -1;
}

/*------------------------------------------------------*/
//���ֽ�����ת����16���Ƶ��ַ���
void byteToHexString(uint8_t *src, uint8_t *dest, uint16_t srclen)
{
    uint16_t i;
    uint8_t a, b;
    for (i = 0; i < srclen; i++)
    {
        a = (src[i] >> 4) & 0x0F;
        b = (src[i]) & 0x0F;
        if (a < 10)
        {
            dest[i * 2] = a + '0';
        }
        else
        {
            dest[i * 2] = a - 10 + 'A';
        }

        if (b < 10)
        {
            dest[i * 2 + 1] = b + '0';

        }
        else
        {
            dest[i * 2 + 1] = b - 10 + 'A';
        }
    }

}
/*------------------------------------------------------*/
static unsigned char  asciiToHex(char ch)
{
    if (ch >= 'a' && ch <= 'f')
    {
        return ch - 'a' + 0x0A;
    }

    if (ch >= 'A' && ch <= 'F')
    {
        return ch - 'A' + 0x0A;
    }

    if (ch >= '0' && ch <= '9')
    {
        return ch - '0';
    }

    if (ch == ' ')
        return 0;
    return 0;
}

//��src��ǰsize*2���ַ���ת����size���ֽ�,����ת����16����
int16_t changeHexStringToByteArray(uint8_t *dest, uint8_t *src, uint16_t size)
{
    uint8_t temp_l, temp_h;
    uint16_t i = 0;

    if (src == NULL || dest == NULL || size == 0)
    {
        return -1;
    }
    for (i = 0; i < (size); i++)
    {
        temp_h = asciiToHex(src[i * 2]);
        temp_l = asciiToHex(src[i * 2 + 1]);
        dest[i] = temp_h << 4 | temp_l;
    }
    return i;
}


//��src��ǰsize*2���ַ���ת����size���ֽ�,����ת����10����
int16_t changeHexStringToByteArray_10in(uint8_t *dest, uint8_t *src, uint16_t size)
{
    uint8_t temp_l, temp_h;
    uint16_t i = 0;

    if (src == NULL || dest == NULL || size == 0)
    {
        return -1;
    }
    for (i = 0; i < (size); i++)
    {
        temp_h = asciiToHex(src[i * 2]) * 10;
        temp_l = asciiToHex(src[i * 2 + 1]);
        temp_h += temp_l;
        dest[i] = temp_h;
    }
    return i;
}
/*------------------------------------------------------*/

void stringToItem(ITEM *item, uint8_t *str, uint16_t len)
{
    uint16_t i, data_len;
    item->item_cnt = 0;
    data_len = 0;
    //���ŷָ�
    memset(item, 0, sizeof(ITEM));
    for (i = 0; i < ITEMCNTMAX; i++)
    {
        item->item_data[i][0] = 0;
    }
    for (i = 0; i < len; i++)
    {
        if (str[i] == ',' || str[i] == '#' || str[i] == '\r' || str[i] == '\n' || str[i] == '=')
        {
            if (item->item_data[item->item_cnt][0] != 0)
            {
                item->item_cnt++;
                data_len = 0;
                if (item->item_cnt >= ITEMCNTMAX)
                {
                    break; ;
                }
            }
        }
        else
        {
            item->item_data[item->item_cnt][data_len] = str[i];
            data_len++;
            if (i + 1 == len)
            {
                item->item_cnt++;
            }
            if (data_len >= ITEMSIZEMAX)
            {
                return ;
            }
            item->item_data[item->item_cnt][data_len] = 0;
        }
    }
}

void strToUppper(char *data, uint16_t len)
{
    //ת��д
    uint8_t i;
    for (i = 0; i < len; i++)
    {
        if (data[i] >= 'a' && data[i] < 'z')
        {
            data[i] = data[i] - 'a' + 'A';
        }
    }
}

void updateRTCtimeRequest(void)
{
    sysinfo.rtcUpdate = 0;
}

void byteArrayInvert(uint8 *data, uint8 dataLen)
{
    uint8 cnt, i, temp;
    if (dataLen <= 1)
    {
        return;
    }
    cnt = dataLen / 2;
    for (i = 0; i < cnt; i++)
    {
        temp = data[i];
        data[i] = data[dataLen - 1 - i];
        data[dataLen - 1 - i] = temp;
    }
}

/**************************************************
@bref		��һ���ַ����еĴ�д��ĸת��Сд
@param
@return
@note
**************************************************/
void stringToLowwer(char *str, uint16_t strlen)
{
    uint16_t i;
    for (i = 0; i < strlen; i++)
    {
        if (str[i] >= 'A' && str[i] <= 'Z')
        {
            str[i] = str[i] - 'A' + 'a';
        }
    }
}

/**************************************************
@bref		��ӡԭʼ�ֽ�����
@param
@return
@note
**************************************************/
void showByteData(uint8_t *mark, uint8_t *buf, uint16_t len)
{
	if (len <= 0 || len > 512)
	{
		LogMessage(DEBUG_ALL, "showByteData==>error");
		return;
	}
	char debug[1025];
	uint16_t debuglen;
	byteToHexString(buf, debug, len);
	debug[len * 2] = 0;
	LogMessage(DEBUG_ALL, mark);
	LogMessageWL(DEBUG_ALL, debug, len*2);
	
}
unsigned char *g_url_encode_buf;
char *url_encode(char const *s, int len, int *new_length)
{
	unsigned char hexchars[] = "0123456789ABCDEF";
    register unsigned char c;
    unsigned char *to, *start;
    unsigned char const *from, *end;
	
    from = (unsigned char *)s;
    end  = (unsigned char *)s + len;	
	unsigned char * url_encode_buf=  (unsigned char *) calloc(1, 120);
	start = to = g_url_encode_buf;

    while (from < end) 
    {
        c = *from++;

        if (c == ' ') 
        {
            *to++ = '+';
        } 
        else if ((c < '0' && c != '-' && c != '.') ||
                 (c < 'A' && c > '9') ||
                 (c > 'Z' && c < 'a' && c != '_') ||
                 (c > 'z')) 
        {
            to[0] = '%';
            to[1] = hexchars[c >> 4];	
            to[2] = hexchars[c & 15];	
            to += 3;
        }
        else 
        {
            *to++ = c;
        }
    }
    *to = 0;
    if (new_length) 
    {
        *new_length = to - start;
    }
    return (char *) start;
}


uint8_t encodeUtf8(uint8_t* buf, int value)
{

	
	//���մ���ֽ���д�뻺����(�͵�ַ��Ӧ������λ)
	if(value <= 0x7f){
		*buf = value & 0x7f;   //��value�����ڴ���
		return 1;
	}	
	else if(value <= 0x7ff){
		//���������utf8��ʽΪ110xxxxx 10xxxxxx
		//���ڵ͵�ַ��д����ֽ�110xxxxx
		//11000000 | ((value & 0111 1100 0000)>>6)
		*buf++ = 0xc0 | ((value & 0x7c0)>>6);
		//���ڸߵ�ַ��д����ֽ�10xxxxxx
		//1000 0000 | (value & 0011 1111)
		*buf = 0x80 | (value & 0x3f);
		return 2;
	}
	else if(value <= 0xffff){
		//���������utf8��ʽΪ1110xxxx 10xxxxxx 10xxxxxx
		//���ڵ͵�ַд����ֽ�1110xxxx
		*buf++ = 0xe0 | ((value & 0xf0000)>>12);
		//Ȼ�����е�ַд�����ֽ�
		*buf++ = 0x80 | ((value & 0xfc0)>>6);
		//����ڸߵ�ַд����ֽ�
		*buf = 0x80 | (value & 0x3f);
		return 3;
	}
	else if(value <= 0x10ffff){
		//�����������utf8��ʽΪ11110xxx 10xxxxxx 10xxxxxx 10xxxxxx
		//���ڵ͵�ַд����ֽ�11110xxx
		*buf++ = 0xf0 | ((value & 0x1c0000)>>18);
		//���ڵڶ��ε�ַд��ڶ����ֽ�10xxxxxx
		*buf++ = 0x80 | ((value & 0x3f00)>>12);
		//Ȼ���ڵ����ε�ַд��������ֽ�10xxxxxx
		*buf++ = 0x80 | ((value & 0xfc0)>>6);
		//����ڸߵ�ַд����ֽ�10xxxxxx
		*buf = 0x80 | (value & 0x3f);
		return 4;
	}
	

	return 0;
}
 
//���ؽ���utf8���ֽ���
uint32_t getByteNumOfDecodeUtf8(uint8_t byte){
	//byteӦ����utf8�����1�ֽڣ����ָ����utf8���������ֽڲ����򷵻�0
	if((byte & 0xc0)==0x80) return 0;  //1000 0000
	if((byte & 0xf8)==0xf0) return 4;  //1111 0000
	if((byte & 0xf0)==0xf0) return 3;  //1110 0000
	if((byte & 0xe0)==0xc0) return 2;  //1100 0000
	return 1;          //ASCII��
}

int enc_unicode_to_utf8_one(unsigned long unic, unsigned char *pOutput, int outSize)
{

    if ( unic <= 0x0000007F )
    {
        // * U-00000000 - U-0000007F:  0xxxxxxx
        *pOutput     = (unic & 0x7F);
        return 1;
    }
    else if ( unic >= 0x00000080 && unic <= 0x000007FF )
    {
        // * U-00000080 - U-000007FF:  110xxxxx 10xxxxxx
        *(pOutput+1) = (unic & 0x3F) | 0x80;
        *pOutput     = ((unic >> 6) & 0x1F) | 0xC0;
        return 2;
    }
    else if ( unic >= 0x00000800 && unic <= 0x0000FFFF )
    {
        // * U-00000800 - U-0000FFFF:  1110xxxx 10xxxxxx 10xxxxxx
        *(pOutput+2) = (unic & 0x3F) | 0x80;
        *(pOutput+1) = ((unic >>  6) & 0x3F) | 0x80;
        *pOutput     = ((unic >> 12) & 0x0F) | 0xE0;
        return 3;
    }
    else if ( unic >= 0x00010000 && unic <= 0x001FFFFF )
    {
        // * U-00010000 - U-001FFFFF:  11110xxx 10xxxxxx 10xxxxxx 10xxxxxx
        *(pOutput+3) = (unic & 0x3F) | 0x80;
        *(pOutput+2) = ((unic >>  6) & 0x3F) | 0x80;
        *(pOutput+1) = ((unic >> 12) & 0x3F) | 0x80;
        *pOutput     = ((unic >> 18) & 0x07) | 0xF0;
        return 4;
    }
    else if ( unic >= 0x00200000 && unic <= 0x03FFFFFF )
    {
        // * U-00200000 - U-03FFFFFF:  111110xx 10xxxxxx 10xxxxxx 10xxxxxx 10xxxxxx
        *(pOutput+4) = (unic & 0x3F) | 0x80;
        *(pOutput+3) = ((unic >>  6) & 0x3F) | 0x80;
        *(pOutput+2) = ((unic >> 12) & 0x3F) | 0x80;
        *(pOutput+1) = ((unic >> 18) & 0x3F) | 0x80;
        *pOutput     = ((unic >> 24) & 0x03) | 0xF8;
        return 5;
    }
    else if ( unic >= 0x04000000 && unic <= 0x7FFFFFFF )
    {
        // * U-04000000 - U-7FFFFFFF:  1111110x 10xxxxxx 10xxxxxx 10xxxxxx 10xxxxxx 10xxxxxx
        *(pOutput+5) = (unic & 0x3F) | 0x80;
        *(pOutput+4) = ((unic >>  6) & 0x3F) | 0x80;
        *(pOutput+3) = ((unic >> 12) & 0x3F) | 0x80;
        *(pOutput+2) = ((unic >> 18) & 0x3F) | 0x80;
        *(pOutput+1) = ((unic >> 24) & 0x3F) | 0x80;
        *pOutput     = ((unic >> 30) & 0x01) | 0xFC;
        return 6;
    }
 
    return 0;
}



