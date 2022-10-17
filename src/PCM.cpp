/**
 ******************************************************************************
 * @file    PCM.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    October 2022
 * @brief   Implementation of PCMClass.
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


/* Includes ------------------------------------------------------------------*/

#include <PCM.h>


/* Class Implementation ------------------------------------------------------*/

/**
 * @brief Constructor
 */
PCMClass::PCMClass()
{
  SampleRate = AUDIO_IN_SAMPLING_FREQUENCY;
  Volume = 100;
  State = PCM_AUDIO_IN_STATE_RESET;
  Buffer = nullptr;
  OnRecv = nullptr;
}

/**
 * @brief  Initialize PCM
 * @retval PCM_OK in case of success, PCM_ERROR otherwise
 */
int PCMClass::Begin()
{
  int ret = PCM_OK;

  if (State != PCM_AUDIO_IN_STATE_RESET) {
    return PCM_ERROR;
  }

  ret = pcm_lowlevel_init(SampleRate);

  if (ret != PCM_ERROR) {
    State = PCM_AUDIO_IN_STATE_STOP;
  } else {
    State = PCM_AUDIO_IN_STATE_ERROR;
  }

  return ret;
}

/**
 * @brief  Deinitialize PCM
 * @retval PCM_OK in case of success, PCM_ERROR otherwise
 */
int PCMClass::End()
{
  int ret = PCM_OK;

  if (State != PCM_AUDIO_IN_STATE_STOP) {
    return PCM_ERROR;
  }

  ret = pcm_lowlevel_deinit();

  OnRecv = nullptr;

  if (ret != PCM_ERROR) {
    State = PCM_AUDIO_IN_STATE_RESET;
  } else {
    State = PCM_AUDIO_IN_STATE_ERROR;
  }

  return ret;
}

/**
 * @brief  Set Volume
 * @param  volume a number from 0 (low) to 100 (high) representing the volume
 * @retval None
 */
void PCMClass::SetVolume(uint8_t volume)
{
  Volume = volume <= 100 ? volume : 100;
}

/**
 * @brief  Get Volume
 * @retval a number from 0 (low) to 100 (high) representing the volume
 */
uint8_t PCMClass::GetVolume()
{
  return Volume;
}

/**
 * @brief  Get State
 * @retval a number representing the state (see PCM_AUDIO_IN_STATE_*)
 */
PCMState_t PCMClass::GetState()
{
  return State;
}

/**
 * @brief  Register the OnRecv callback used by HAL half and full completed callbacks
 * @retval None
 */
void PCMClass::OnReceive(void(*fun)())
{
  OnRecv = fun;
}

/**
 * @brief  Start recording PCM audio to buf
 * @retval PCM_OK in case of success, PCM_ERROR otherwise
 */
int PCMClass::Record(uint16_t *buf)
{
  int ret = PCM_OK;

  if (State != PCM_AUDIO_IN_STATE_STOP || OnRecv == nullptr || buf == nullptr) {
    return PCM_ERROR;
  }

  Buffer = buf;

  ret = pcm_lowlevel_start();

  if (ret != PCM_ERROR) {
    State = PCM_AUDIO_IN_STATE_RECORDING;
  } else {
    State = PCM_AUDIO_IN_STATE_ERROR;
  }

  return ret;
}

/**
 * @brief  Pause recording
 * @retval PCM_OK in case of success, PCM_ERROR otherwise
 */
int PCMClass::Pause()
{
  int ret = PCM_OK;

  if (State != PCM_AUDIO_IN_STATE_RECORDING) {
    return PCM_ERROR;
  }

  ret = pcm_lowlevel_stop();

  if (ret != PCM_ERROR) {
    State = PCM_AUDIO_IN_STATE_PAUSE;
  } else {
    State = PCM_AUDIO_IN_STATE_ERROR;
  }

  return ret;
}

/**
 * @brief  Resume recording
 * @retval PCM_OK in case of success, PCM_ERROR otherwise
 */
int PCMClass::Resume()
{
  int ret = PCM_OK;

  if (State != PCM_AUDIO_IN_STATE_PAUSE) {
    return PCM_ERROR;
  }

  ret = pcm_lowlevel_start();

  if (ret != PCM_ERROR) {
    State = PCM_AUDIO_IN_STATE_RECORDING;
  } else {
    State = PCM_AUDIO_IN_STATE_ERROR;
  }

  return ret;
}

/**
 * @brief  Stop recording
 * @retval PCM_OK in case of success, PCM_ERROR otherwise
 */
int PCMClass::Stop()
{
  int ret = PCM_OK;

  if (State != PCM_AUDIO_IN_STATE_RECORDING) {
    return PCM_ERROR;
  }

  ret = pcm_lowlevel_stop();

  Buffer = nullptr;

  if (ret != PCM_ERROR) {
    State = PCM_AUDIO_IN_STATE_STOP;
  } else {
    State = PCM_AUDIO_IN_STATE_ERROR;
  }

  return ret;
}

__attribute__((weak)) int pcm_lowlevel_init(uint32_t sampleRate)
{
  return PCM_ERROR;
}

__attribute__((weak)) int pcm_lowlevel_deinit()
{
  return PCM_ERROR;
}

__attribute__((weak)) int pcm_lowlevel_start()
{
  return PCM_ERROR;
}

__attribute__((weak)) int pcm_lowlevel_stop()
{
  return PCM_ERROR;
}

PCMClass PCM;
