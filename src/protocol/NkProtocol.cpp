#include "NkProtocol.h"

void SerialResponseWriter::print(const char* value) { Serial.print(value); }
void SerialResponseWriter::print(const String& value) { Serial.print(value); }
void SerialResponseWriter::print(int value) { Serial.print(value); }
void SerialResponseWriter::print(unsigned int value) { Serial.print(value); }
void SerialResponseWriter::print(unsigned long value) { Serial.print(value); }
void SerialResponseWriter::println() { Serial.println(); }

void nk4WriteOk(IResponseWriter& writer, const String& seq, const String& fields)
{
  writer.print("NK4 seq=");
  writer.print(seq.length() > 0 ? seq : "0");
  writer.print(" ok");
  if (fields.length() > 0)
  {
    writer.print(" ");
    writer.print(fields);
  }
  writer.println();
}

void nk4WriteError(IResponseWriter& writer, const String& seq, const char* code, const char* message)
{
  writer.print("NK4 seq=");
  writer.print(seq.length() > 0 ? seq : "0");
  writer.print(" err code=");
  writer.print(code != NULL ? code : "internal_error");
  writer.print(" msg=");
  writer.print(message != NULL ? message : "error");
  writer.println();
}

String nk4GetValue(const NkCommand& command, const char* key)
{
  for (uint8_t i = 0; i < command.pairCount; i++)
  {
    if (command.pairs[i].key == key)
    {
      return command.pairs[i].value;
    }
  }
  return "";
}

bool nk4HasKey(const NkCommand& command, const char* key)
{
  for (uint8_t i = 0; i < command.pairCount; i++)
  {
    if (command.pairs[i].key == key)
    {
      return true;
    }
  }
  return false;
}

bool parseNk4Line(const String& line, NkCommand* outCommand, String* errorCode, String* errorMessage)
{
  if (outCommand == NULL)
  {
    return false;
  }

  *outCommand = NkCommand();
  String input = line;
  input.trim();
  if (!input.startsWith("NK4"))
  {
    if (errorCode != NULL) *errorCode = "invalid_command";
    if (errorMessage != NULL) *errorMessage = "expected_NK4";
    return false;
  }

  int start = 3;
  while (start < input.length())
  {
    while (start < input.length() && input[start] == ' ')
    {
      start++;
    }
    if (start >= input.length())
    {
      break;
    }

    int space = input.indexOf(' ', start);
    String token = (space >= 0) ? input.substring(start, space) : input.substring(start);
    int equals = token.indexOf('=');
    if (equals <= 0 || equals == token.length() - 1)
    {
      if (errorCode != NULL) *errorCode = "invalid_key";
      if (errorMessage != NULL) *errorMessage = "bad_token";
      return false;
    }

    String key = token.substring(0, equals);
    String value = token.substring(equals + 1);
    key.toLowerCase();
    if (key == "seq")
    {
      outCommand->seq = value;
    }
    else if (key == "cmd")
    {
      value.toLowerCase();
      outCommand->command = value;
    }
    else
    {
      if (outCommand->pairCount >= (sizeof(outCommand->pairs) / sizeof(outCommand->pairs[0])))
      {
        if (errorCode != NULL) *errorCode = "range_error";
        if (errorMessage != NULL) *errorMessage = "too_many_keys";
        return false;
      }
      outCommand->pairs[outCommand->pairCount].key = key;
      outCommand->pairs[outCommand->pairCount].value = value;
      outCommand->pairCount++;
    }

    if (space < 0)
    {
      break;
    }
    start = space + 1;
  }

  if (outCommand->seq.length() == 0)
  {
    if (errorCode != NULL) *errorCode = "invalid_key";
    if (errorMessage != NULL) *errorMessage = "missing_seq";
    return false;
  }
  if (outCommand->command.length() == 0)
  {
    if (errorCode != NULL) *errorCode = "invalid_command";
    if (errorMessage != NULL) *errorMessage = "missing_cmd";
    return false;
  }

  return true;
}
