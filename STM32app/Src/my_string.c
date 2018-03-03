#include <inttypes.h>
#include "my_string.h"

/** \brief Compare two strings
	  * \param str1		string #1
	  * \param str2		string #2
      * \return 1 if strings are equal, 0 if not
      */
uint8_t compare_strings(const char* str1, const char* str2)
{
	uint16_t i=0;
	while(str1[i] != '\0')
	{
		if(str1[i] != str2[i]) return 0;
		i++;
	}

	if(str1[i] != str2[i]) return 0; ///< Compare NULLs
	else return 1;

}

/** \brief Checks if one string occur in second one
	  * \param look_here		string to be searched
	  * \param for_this			wanted string
      * \return pointer to first occurence of wanted string. If not found returns NULL pointer
      */
char* str_str(char* look_here, char* for_this)
{
	uint16_t j,i=0;
	while(look_here[i])
	{
		if(look_here[i]==for_this[0])
		{
			j=i;
			while(look_here[j] == for_this[j-i])
			{
				if(look_here[j] == '\0' && i==0) return &look_here[0];
				j++;
			}

			if(for_this[j-i]== '\0') return &look_here[i];
			else if(look_here[j] == '\0') return 0;
		}

		i++;
	}
	return 0;
}

/** \brief Calculates null-terminated string length
	  * \param str		null-terminated string with unknown length
      * \return string length without NULL sign, e.g strlen("1234") returns 4
      */
uint16_t str_len(const char* str)
{
	uint16_t len = 0;
	while(*str)
	{
		len++;
		str++;
	}
	return len;
}



/** \brief Convert null-terminated array filled with digits to number
	  * \param str		null-terminated string containing number digits
      * \return number created from concatenation of digits stored in 'str' array
      */
uint16_t string_to_num(const char* str)
{
	uint16_t len = str_len(str);
	int8_t i;
	uint16_t magnitude=1;
	uint16_t result=0;

	for(i=len-1 ; i>=0 ; i--)
	{
		result = result + magnitude*(str[i]-'0');
		magnitude *= 10;
	}

	return result;
}
