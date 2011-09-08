 /*
 * Copyright (C) 2009 by Ulrich Friedrich Klank <klank@in.tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "pluginlib/class_list_macros.h"

/*Includes for Reading Plugins*/

/*interface of cognitive_perception*/

#include "IplImageReading.h"

#include "FaceDetection.h"

#include "DetectedFace.h"

#include "ImageSubscription.h"

using namespace cop;
PLUGINLIB_REGISTER_CLASS(ImageSubscription, cop::ImageSubscription, Sensor);

PLUGINLIB_REGISTER_CLASS(IplImageReading, cop::IplImageReading, Reading);

PLUGINLIB_REGISTER_CLASS(FaceDetection, cop::FaceDetection, LocateAlgorithm);

PLUGINLIB_REGISTER_CLASS(DetectedFace, cop::DetectedFace, Descriptor);
