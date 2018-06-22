/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
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

#include "behavior.hpp"
#include <iostream>
#include <time.h>             

Behavior::Behavior() noexcept:
  m_frontUltrasonicReading{},
  m_rearUltrasonicReading{},
  m_leftIrReading{},
  m_rightIrReading{},
  m_groundSteeringAngleRequest{},
  m_pedalPositionRequest{},
  m_frontUltrasonicReadingMutex{},
  m_rearUltrasonicReadingMutex{},
  m_leftIrReadingMutex{},
  m_rightIrReadingMutex{},
  m_groundSteeringAngleRequestMutex{},
  m_pedalPositionRequestMutex{},
  m_aMutex{},
  m_bMutex{},
  m_flag1{},
  m_time{},
  m_a{},
  m_b{}
{
}

opendlv::proxy::GroundSteeringRequest Behavior::getGroundSteeringAngle() noexcept
{
  std::lock_guard<std::mutex> lock(m_groundSteeringAngleRequestMutex);
  return m_groundSteeringAngleRequest;
}

opendlv::proxy::PedalPositionRequest Behavior::getPedalPositionRequest() noexcept
{
  std::lock_guard<std::mutex> lock(m_pedalPositionRequestMutex);
  return m_pedalPositionRequest;
}

void Behavior::setFrontUltrasonic(opendlv::proxy::DistanceReading const &frontUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_frontUltrasonicReadingMutex);
  m_frontUltrasonicReading = frontUltrasonicReading;
}

void Behavior::setRearUltrasonic(opendlv::proxy::DistanceReading const &rearUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rearUltrasonicReadingMutex);
  m_rearUltrasonicReading = rearUltrasonicReading;
}

void Behavior::setLeftIr(opendlv::proxy::VoltageReading const &leftIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_leftIrReadingMutex);
  m_leftIrReading = leftIrReading;
}

void Behavior::setRightIr(opendlv::proxy::VoltageReading const &rightIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rightIrReadingMutex);
  m_rightIrReading = rightIrReading;
}
void Behavior::setDist(float const &a) noexcept
{
  std::lock_guard<std::mutex> lock(m_aMutex);
  m_a = a;
}

void Behavior::setAngle(float const &b) noexcept
{
  std::lock_guard<std::mutex> lock(m_bMutex);
  m_b = b;
}


void Behavior::step(float currentTime) noexcept
{
  opendlv::proxy::DistanceReading frontUltrasonicReading;
  opendlv::proxy::DistanceReading rearUltrasonicReading;
  opendlv::proxy::VoltageReading leftIrReading;
  opendlv::proxy::VoltageReading rightIrReading;
  float distance_car;
  float angle_car;
  {
    std::lock_guard<std::mutex> lock1(m_frontUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock2(m_rearUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock3(m_leftIrReadingMutex);
    std::lock_guard<std::mutex> lock4(m_rightIrReadingMutex);
    std::lock_guard<std::mutex> lock5(m_aMutex);
    std::lock_guard<std::mutex> lock6(m_bMutex);

    frontUltrasonicReading = m_frontUltrasonicReading;
    rearUltrasonicReading = m_rearUltrasonicReading;
    leftIrReading = m_leftIrReading;
    rightIrReading = m_rightIrReading;
    distance_car = m_a;
    angle_car = m_b;
  }

  float frontDistance = frontUltrasonicReading.distance();
  float rearDistance = rearUltrasonicReading.distance();
  double leftDistance = convertIrVoltageToDistance(leftIrReading.voltage());
  double rightDistance = convertIrVoltageToDistance(rightIrReading.voltage());

  //Behavior:
  float pedalPosition = 0.12f;
  float groundSteeringAngle = -0.2f;
  if (angle_car<0.00001f){groundSteeringAngle = 0.2f;}
  if (angle_car<0.2f && angle_car>-0.2f){groundSteeringAngle = 0.0f;}

  groundSteeringAngle = (float)angle_car*(-0.4f);
  if (angle_car>-0.03f && angle_car<0.03f){distance_car = frontDistance*100.0f+12.0f;}

  if (distance_car>220.0f) {pedalPosition = 0.18f; }
  if (distance_car>137.0f) {pedalPosition = 0.14f; }
  if (distance_car<50.0f) {pedalPosition = 0.00f; }
  if (distance_car<24.0f) {pedalPosition = -0.42f; groundSteeringAngle = 0.0f;}
  if (distance_car<5.0f) {pedalPosition = -0.44f; groundSteeringAngle = 0.0f;}

  if (rightDistance < 0.35f) { groundSteeringAngle = 0.35f;}
  if (leftDistance < 0.35f) { groundSteeringAngle = -0.35f;}
  if (frontDistance < 0.2f ) { pedalPosition = -0.43f; groundSteeringAngle = 0.0f; }
  if (rearDistance < 0.22f ) { pedalPosition = 0.0f; groundSteeringAngle = 0.0f; }


  float delta_time = ((float)(clock()))/CLOCKS_PER_SEC;
  delta_time = delta_time - currentTime/10.0f;

  std::cout << "Time"<<delta_time<< std::endl;
  std::cout << "Angle car "<<angle_car << std::endl;
  std::cout << "Distance car "<<distance_car << std::endl;

  {
    std::lock_guard<std::mutex> lock1(m_groundSteeringAngleRequestMutex);
    std::lock_guard<std::mutex> lock2(m_pedalPositionRequestMutex);

    opendlv::proxy::GroundSteeringRequest groundSteeringAngleRequest;
    groundSteeringAngleRequest.groundSteering(groundSteeringAngle);
    m_groundSteeringAngleRequest = groundSteeringAngleRequest;

    opendlv::proxy::PedalPositionRequest pedalPositionRequest;
    pedalPositionRequest.position(pedalPosition);
    m_pedalPositionRequest = pedalPositionRequest;
  }
}

double Behavior::convertIrVoltageToDistance(float voltage) const noexcept
{
  double voltageGain1 = 35.7163;
  double voltageGain2 = -79.2321;
  double voltageGain3 = 45.1424;

  double distance = voltageGain1*(voltage*voltage) + voltageGain2*voltage + voltageGain3;
  distance = distance/100.0f;
  return distance;
}
