// SPDX-License-Identifier: Apache-2.0

/*
 * Copyright (c) 2024 Witold Olechowski.
 */

#include "utils.h"

/**************************************************************************/
/*!
	@brief  Converts angle to azimuth string

	@params[in]
				Double unit : working with angles in degree
	@returns
				returns one of the 16 compass azimuth
*/
/**************************************************************************/
char *degreeToCompass(double angle)
{
  if (angle < 0.0)
    return "ERROR";
  else if (angle < 11.25)
    return "N";
  else if (angle < 33.75)
    return "NNE";
  else if (angle < 56.25)
    return "NE";
  else if (angle < 78.75)
    return "ENE";
  else if (angle < 101.25)
    return "E";
  else if (angle < 123.75)
    return "ESE";
  else if (angle < 146.25)
    return "SE";
  else if (angle < 168.75)
    return "SSE";
  else if (angle < 191.25)
    return "S";
  else if (angle < 213.75)
    return "SSW";
  else if (angle < 236.25)
    return "SW";
  else if (angle < 258.75)
    return "WSW";
  else if (angle < 281.25)
    return "W";
  else if (angle < 303.75)
    return "WNW";
  else if (angle < 326.25)
    return "NW";
  else if (angle < 348.75)
    return "NNW";
  else if (angle <= 360.0)
    return "N";

  return "ERROR";
}
