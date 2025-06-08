/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef PREFERENCESDIALOGROS_H_
#define PREFERENCESDIALOGROS_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <rtabmap/gui/PreferencesDialog.h>

using namespace rtabmap;

class PreferencesDialogROS : public PreferencesDialog
{
	Q_OBJECT
public:
	PreferencesDialogROS(rclcpp::Node * node, const QString & configFile, const std::string & rtabmapNodeName);
	virtual ~PreferencesDialogROS();

	virtual QString getIniFilePath() const;
	virtual QString getTmpIniFilePath() const;
	bool hasAllParameters();

public Q_SLOTS:
	void readRtabmapNodeParameters();

protected:
	virtual QString getParamMessage();

	virtual void readCameraSettings(const QString & filePath);
	virtual bool readCoreSettings(const QString & filePath);
	virtual void writeCameraSettings(const QString &) const {}
	virtual void writeCoreSettings(const QString & filePath) const;

private:
	QString configFile_;
	rclcpp::Node * node_;
	std::string rtabmapNodeName_;
};

#endif /* PREFERENCESDIALOGROS_H_ */
