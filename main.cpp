#pragma execution_character_set("utf-8")

#include "registration.h"
#include <QtWidgets/QApplication>

extern "C" __declspec(dllexport) bool GetPluginInstance(wl::LidarDataModel *data_model)
{
	Registration *registration = new Registration;

	if (registration->exec() == QDialog::Accepted)
	{
		delete registration; //Ã»¿´¶®
		registration = NULL;
	}

	return true;
}


extern "C" __declspec(dllexport) int GetPluginID()
{
	return 00000001;
}

