/**
 ******************************************************************************
 * @file    PCM.h
 * @author  SRA
 * @version V1.0.0
 * @date    October 2022
 * @brief   Interface of PCMClass.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2022 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef PCM_CLASS_H
#define PCM_CLASS_H


/* Includes ------------------------------------------------------------------*/

#include <Arduino.h>
#include <pcm_config.h>


/* Defines -------------------------------------------------------------------*/


/* Typedefs ------------------------------------------------------------------*/
typedef enum PCMState_e {
  PCM_AUDIO_IN_STATE_RESET = 0,
  PCM_AUDIO_IN_STATE_STOP,
  PCM_AUDIO_IN_STATE_RECORDING,
  PCM_AUDIO_IN_STATE_PAUSE,
  PCM_AUDIO_IN_STATE_ERROR
} PCMState_t;


/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class for handling PCM audio.
 */
class PCMClass {
  public:
    PCMClass();

    int Begin();
    int End();
    void SetVolume(uint8_t volume);
    uint8_t GetVolume();
    PCMState_t GetState();
    void OnReceive(void(*fun)());
    int Record(uint16_t *buf);
    int Pause();
    int Resume();
    int Stop();

  private:
    uint32_t   SampleRate;    /* sample rate           */
    uint32_t   Volume;        /* volume                */
    PCMState_t State;         /* current state         */
    uint16_t   *Buffer;       /* record buffer         */
    void (*OnRecv)();         /* recv callback         */

    friend void HAL_MDF_AcqCpltCallback(MDF_HandleTypeDef *);
    friend void HAL_MDF_AcqHalfCpltCallback(MDF_HandleTypeDef *);
};

extern PCMClass PCM;

#endif /* PCM_CLASS_H */
