/* Copyright 2016 CyberTech Labs Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * This file was modified by Yurii Litvinov to make it comply with the requirements of trikRuntime
 * project. See git revision history for detailed changes. */

#pragma once

#include <QtCore/QObject>

#include <trikControl/vectorSensorInterface.h>
#include <trikControl/motorInterface.h>
#include <trikControl/displayInterface.h>

class AccelerometerListener : public QObject
{
	Q_OBJECT

public:
	explicit AccelerometerListener(
			trikControl::VectorSensorInterface &accelerometer
			, trikControl::MotorInterface &motor1
			, trikControl::MotorInterface &motor2
			, trikControl::DisplayInterface &display);

public slots:
	void onTimeout();

private:
	trikControl::VectorSensorInterface &mAccelerometer;
	trikControl::MotorInterface &mMotor1;
	trikControl::MotorInterface &mMotor2;
	trikControl::DisplayInterface &mDisplay;
};
