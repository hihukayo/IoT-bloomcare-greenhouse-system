/**
  ******************************************************************************
  * @file    wifi_secrets.example.h
  * @brief   The template of the one file that carries a password.
  * @note    Copy it to wifi_secrets.h and fill in the access point of the bench.
  *          The copy is listed in .gitignore, so it stays on the machine it was
  *          made on and never reaches the repository.
  * @note    Both values may stay empty. The gateway then waits for its setup,
  *          the status bar keeps saying WIFI -- and the network screen of the
  *          panel says OFFLINE.
  * @note    The radio of the ESP32-S3 does 2.4 GHz only: a 5 GHz access point is
  *          not found, no matter what is written here.
  ******************************************************************************/
#ifndef __WIFI_SECRETS_H
#define __WIFI_SECRETS_H

/** Name of the access point. */
#define WIFI_SSID       ""

/** Passphrase of that access point, empty for an open network. */
#define WIFI_PASS       ""

#endif /* __WIFI_SECRETS_H */