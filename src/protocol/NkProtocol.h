#pragma once

#include <Arduino.h>

class IResponseWriter
{
public:
  virtual void print(const char* value) = 0;
  virtual void print(const String& value) = 0;
  virtual void print(int value) = 0;
  virtual void print(unsigned int value) = 0;
  virtual void print(unsigned long value) = 0;
  virtual void println() = 0;
};

class SerialResponseWriter : public IResponseWriter
{
public:
  void print(const char* value) override;
  void print(const String& value) override;
  void print(int value) override;
  void print(unsigned int value) override;
  void print(unsigned long value) override;
  void println() override;
};

struct NkKeyValue
{
  String key;
  String value;
};

struct NkCommand
{
  String seq;
  String command;
  NkKeyValue pairs[18];
  uint8_t pairCount = 0;
};

String nk4GetValue(const NkCommand& command, const char* key);
bool nk4HasKey(const NkCommand& command, const char* key);
bool parseNk4Line(const String& line, NkCommand* outCommand, String* errorCode, String* errorMessage);
void nk4WriteOk(IResponseWriter& writer, const String& seq, const String& fields);
void nk4WriteError(IResponseWriter& writer, const String& seq, const char* code, const char* message);
