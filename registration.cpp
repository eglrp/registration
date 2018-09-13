#include "registration.h"

#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>
#include <QtGui/QDesktopServices>
#include <QtCore/QUrl>

#include "DataGenerator.h"

Registration::Registration(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	connect(ui.pushButton, SIGNAL(clicked()), this, SLOT(setpath_before()));
	connect(ui.pushButton_2, SIGNAL(clicked()), this, SLOT(setpath_after()));
	connect(ui.pushButton_6, SIGNAL(clicked()), this, SLOT(setpath_out()));
	connect(ui.pushButton_5, SIGNAL(clicked()), this, SLOT(DoRegistrate()));
}

//
void Registration::setpath_before()
{
	if (!file_after_.isNull())
	{
		QFileInfo info(file_after_);

		QString fileAfterPath = info.path();

		file_before_ = QFileDialog::getOpenFileName(qobject_cast<QWidget *>(this->parent()), tr("ѡ���·��"), fileAfterPath, tr("Point Cloud Files (*.las)"));
	}
	else
	{
		file_before_ = QFileDialog::getOpenFileName(qobject_cast<QWidget *>(this->parent()), tr("ѡ���·��"), "", tr("Point Cloud Files (*.las)"));
	}

	if (!file_before_.isNull())
	{
		ui.lineEdit->setText(file_before_);
	}
}

//
void Registration::setpath_after()
{
	if (!file_before_.isNull())
	{
		QFileInfo info(file_before_);

		QString fileBeforePath = info.path();

		file_after_ = QFileDialog::getOpenFileName(qobject_cast<QWidget *>(this->parent()), tr("ѡ���·��"), fileBeforePath, tr("Point Cloud Files (*.las)"));
	}
	else
	{
		file_after_ = QFileDialog::getOpenFileName(qobject_cast<QWidget *>(this->parent()), tr("ѡ���·��"), "", tr("Point Cloud Files (*.las)"));
	}

	if (!file_after_.isNull())
	{
		ui.lineEdit_2->setText(file_after_);
	}
}

//
void Registration::setpath_out()
{
	if (!file_before_.isNull())
	{
		QFileInfo info(file_before_);

		QString fileBeforePath = info.path();

		file_out_ = QFileDialog::getSaveFileName(qobject_cast<QWidget *>(this->parent()), tr("ѡ�񱣴�·��"), fileBeforePath, tr("Point Cloud Files (*.las)"));
	}
	else
	{
		file_out_ = QFileDialog::getSaveFileName(qobject_cast<QWidget *>(this->parent()), tr("ѡ�񱣴�·��"), "", tr("Point Cloud Files (*.las)"));
	}

	if (!file_out_.isNull())
	{
		ui.lineEdit_10->setText(file_out_);
	}
}

//
void Registration::DoRegistrate()
{
	if (ui.lineEdit->text().isEmpty() || ui.lineEdit_2->text().isEmpty() || ui.lineEdit_10->text().isEmpty())
	{
		QMessageBox::StandardButton reply = QMessageBox::warning(qobject_cast<QWidget *>(this->parent()), tr("��ʾ"), tr("���������·����"), QMessageBox::Yes);

		if (reply == QMessageBox::Yes)
		{
			return;
		}
	}
	else
	{
		this->setVisible(false);
	}

	//
	QFileInfo info(file_out_);

	QString fileOutPath = "file:///" + info.path();

	//
	DataGenerator dataGenerator;

	dataGenerator.setFileName(ui.lineEdit->text(), ui.lineEdit_2->text());

	dataGenerator.setRansacLineParameters(ui.lineEdit_5->text().toDouble(),
										  ui.lineEdit_6->text().toInt());

	dataGenerator.setIcpParameters(ui.lineEdit_7->text().toInt(),
								   ui.lineEdit_8->text().toDouble(),
								   ui.lineEdit_9->text().toDouble());

	dataGenerator.setFileNameOut(ui.lineEdit_10->text());

	bool hasRegistrated;

	hasRegistrated = dataGenerator.DoRegistrate();

	//
	if (hasRegistrated)
	{
		QMessageBox message(QMessageBox::NoIcon, tr("��ʾ"), tr("�Ƿ������ļ������ļ��У�"), QMessageBox::Yes | QMessageBox::No, NULL);

		if (message.exec() == QMessageBox::Yes)
		{
			QDesktopServices::openUrl(QUrl(fileOutPath, QUrl::TolerantMode));
		}
	}
	else
	{
		QMessageBox message(QMessageBox::NoIcon, tr("��ʾ"), tr("�������ѷ������ݣ�"), QMessageBox::Yes, NULL);

		if (message.exec() == QMessageBox::Yes)
		{
			return;
		}
	}
}