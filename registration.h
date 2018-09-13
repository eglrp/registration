#pragma execution_character_set("utf-8")

#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <QtWidgets/QDialog>
#include "ui_registration.h"

#include "data_model.h"

class Registration : public QDialog
{
	Q_OBJECT

public:
	Registration(QWidget *parent = Q_NULLPTR);

private:
	Ui::RegistrationClass ui;

public slots:
	void setpath_before();
	void setpath_after();
	void setpath_out();
	void DoRegistrate();

private:
	QString file_before_;
	QString file_after_;
	QString file_out_;
};

#endif