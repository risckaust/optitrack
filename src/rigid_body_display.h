/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RIGID_BODY_DISPLAY_H
#define RIGID_BODY_DISPLAY_H

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#include <optitrack/RigidBodyArray.h>
#endif

namespace rviz
{
class BoolProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
class StringProperty;
class VectorProperty;
}

namespace optitrack
{

/**
 * \class RigidBodyDisplay
 * \brief Displays rigid body markers from optitrack::RigidBodyArray messages.
 */
class RigidBodyDisplay: public rviz::MessageFilterDisplay<optitrack::RigidBodyArray>
{
Q_OBJECT
public:
  RigidBodyDisplay();
  virtual ~RigidBodyDisplay();

  /** @brief Overridden from Display. */
  virtual void reset();

protected:
  /** @brief Overridden from Display. */
  virtual void onInitialize();

  /** @brief Overridden from MessageFilterDisplay. */
  virtual void processMessage( const optitrack::RigidBodyArray::ConstPtr& msg );

private Q_SLOTS:
  void updateSink();

private:
  rviz::BoolProperty*   fit_property_;
  rviz::BoolProperty*   show_hull_property_;
  rviz::BoolProperty*   show_plane_property_;
  rviz::ColorProperty*  color_property_;
  rviz::FloatProperty*  diameter_property_;
  rviz::StringProperty* hull_property_;
  rviz::IntProperty*    id_property_;
  rviz::IntProperty*    selected_property_;
  rviz::VectorProperty* normal_property_;
  rviz::StringProperty* plane_property_;
};

} // namespace range_plugin

#endif /* RIGID_BODY_DISPLAY_H */