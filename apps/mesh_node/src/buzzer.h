/**
 * @file buzzer.h
 * @brief Notification sounds on the ZBook piezo buzzer.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef BUZZER_H_
#define BUZZER_H_

#include <stdbool.h>

void buzzer_set_enabled(bool on);
bool buzzer_enabled(void);

#endif /* BUZZER_H_ */
