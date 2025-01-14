#ifndef HTTP_H_
#define HTTP_H_

#ifndef HTTP_FORM_FIELDS_MAX
# define HTTP_FORM_FIELDS_MAX 32
#endif

// Class implements basic HTTP protocol parsing
class HttpRequest
{
public:
  // constructs new HTTP parser
  HttpRequest()
  {
    lastLine = "";
    lastMethod = "";
    lastUrl = "";
    state = 0x00;
    argsCount = 0;
    bytesToRead = 0;
  }

  // parses incoming stream. when receives HTTP requests, returns true
  bool parse(char c)
  {
    // end of line reached, process
    switch(state)
    {
      case 0x00:
        // waiting for GET / POST header
        if(c == '\r')
        {
          // end of line received
          if(lastLine.indexOf("HTTP/1.1") >= 0)
          {
            // request is an HTTP request {METHOD} {URL} {PROTOCOL}
            int idx = 0;
            const int len = lastLine.length();
            char tmp;
  
            // parse method
            lastMethod = "";
            while(idx < len && (tmp = lastLine[idx++]) != ' ')
              lastMethod += tmp;
            lastMethod.toUpperCase();
              
            // parse url
            lastUrl = "";
            while(idx < len && (tmp = lastLine[idx++]) != ' ' && tmp != '?')
              lastUrl += tmp;
  
            argsCount = 0;
            if(tmp == '?')
              parseArgs(lastLine, idx+1);

            bytesToRead = 0;
            state = 0x01;
          }
          else
          {
            // reset line
            lastLine = "";
          }
        }
        else if(c != '\n')
        {
          // store byte
          lastLine += c;
        }
        break;
        
      case 0x01:
        // reading HTTP headers - take Content-Length
        if(c == '\r')
        {
          // end of line received
          lastLine.toLowerCase();

          if(lastLine.length() == 0)
          {
            // empty line is a separator of headers from the body
            if(bytesToRead > 0)
            {
              // expecting some body
              state = 0x02;
            }
            else
            {
              state = 0x00;
              return true;
            }
          }
          else if(lastLine.startsWith("content-length:"))
          {
            String val = lastLine.substring(15);
            val.trim();
            
            bytesToRead = val.toInt();
          }

          // reset line
          lastLine = "";
        }
        else if(c != '\n')
        {
          // store byte
          lastLine += c;
        }
        break;

      case 0x02:
        // reading body
        if(lastLine.length() == bytesToRead)
        {
          // proces body
          parseArgs(lastLine);
          state = 0x00;
          lastLine = "";
          return true;
        }
        else
        {
          // store byte
          lastLine += c;
        }
        break;
        
      default:
        state = 0x00;
        lastLine = "";  
    }
    return false;
  }

  // Returns last request method
  const String& method() { return lastMethod; }

  // Returns last request url
  const String& url() { return lastUrl; }

  // Returns value of given form field
  String arg(const String& name)
  {
    for(int i=0; i < argsCount; i++)
    {
      const FormField& field = lastArgs[i];
      if(field.name == name)
        return field.val;
    }
    return "";
  }
  
private:
  // structure representing form argument
  struct FormField
  {
    String name;
    String val;
  };
  
  byte state;
  String lastLine;
  String lastMethod;
  String lastUrl;
  FormField lastArgs[HTTP_FORM_FIELDS_MAX];
  int argsCount;
  int bytesToRead;

  // parses form fields (both body and query)
  void parseArgs(const String& str, int startIdx = 0)
  {
    int idx = startIdx, midIdx = startIdx;
    char c;
    
    do
    {
      if(argsCount >= HTTP_FORM_FIELDS_MAX)
          return; // overflow

      c = str[idx];
      
      if(c == '=')
      {
        // end of the name reached, store mid index
        midIdx = idx;
      }
      else if(c == '&' || c == '\r' || c == '\n' || c == ' ' || c == '\0')
      {
        // end of the item reached, store
        if(midIdx > startIdx)
        { 
          FormField& field = lastArgs[argsCount++];
          field.name = str.substring(startIdx, midIdx);
          field.val = str.substring(midIdx+1, idx);
        }
        
        // reset indices
        startIdx = idx+1;
      }

      ++idx;
    }
    while(c != '\0');
  }
};

#endif
