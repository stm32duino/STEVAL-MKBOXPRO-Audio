# STEVAL Audio

Arduino library to support the [MP23DB01HP](https://www.st.com/en/mems-and-sensors/mp23db01hp.html) digital microphone.
Currently, this library works only with the STEVAL board.
It requires a [STM32 Core](https://github.com/stm32duino/Arduino_Core_STM32) equal to or greater than version X.Y.Z.

## API

This library can be used to record PCM audio on STEVAL board.

After including `PCM.h`, initialize the library by calling `PCM.Begin`:

```cpp
#include <PCM.h>

void setup()
{
  // [...]
  PCM.Begin();
  // [...]
}
```

Register the data processing callback using `PCM.OnReceive`:

```cpp
void process()
{
  Serial.write((const uint8_t *)RecBuff, PCM_REC_BUFF_SIZE * 2U);
}

void setup()
{
  // [...]
  PCM.OnReceive(process);
  // [...]
}
```

Start recording PCM audio by passing a non-null pointer of a `uint16_t[PCM_REC_BUFF_SIZE]` buffer to `PCM.Record`:

```cpp
uint16_t RecBuff[PCM_REC_BUFF_SIZE];

void setup()
{
  // [...]
  PCM.Record(RecvBuff);
  // [...]
}
```

You can pause a recording by calling `PCM.Pause`; in order to resume it, just call `PCM.Resume`:

```cpp
void loop()
{
  // [...]
  if (cmd == '1') PCM.Pause();
  if (cmd == '2') PCM.Resume();
  // [...]
}
```

In order to deinitialize the library, you can execute `PCM.End` (if recording, you need first to stop it):

```cpp
void loop()
{
  // [...]
  if (cmd == '3') {
    if (PCM.GetState() == PCM_AUDIO_IN_STATE_RECORDING) PCM.Stop();
    PCM.End();
    while(1);
  }
  // [...]
}
```

## More Info

Source files can be found at
https://github.com/stm32duino/STEVAL-Audio

Take a look at STEVAL examples applications at
https://github.com/stm32duino/STEVAL-Examples
