/********************************************************************************
** Form generated from reading UI file 'registration.ui'
**
** Created by: Qt User Interface Compiler version 5.5.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_REGISTRATION_H
#define UI_REGISTRATION_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_RegistrationClass
{
public:
    QVBoxLayout *verticalLayout_5;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *lineEdit;
    QPushButton *pushButton;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QLineEdit *lineEdit_2;
    QPushButton *pushButton_2;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_4;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox_3;
    QHBoxLayout *horizontalLayout_6;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_5;
    QLineEdit *lineEdit_5;
    QLabel *label_6;
    QLineEdit *lineEdit_6;
    QGroupBox *groupBox_4;
    QHBoxLayout *horizontalLayout_8;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_7;
    QLineEdit *lineEdit_7;
    QLabel *label_8;
    QLineEdit *lineEdit_8;
    QLabel *label_9;
    QLineEdit *lineEdit_9;
    QGroupBox *groupBox_5;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_3;
    QLineEdit *lineEdit_10;
    QPushButton *pushButton_6;
    QPushButton *pushButton_5;

    void setupUi(QDialog *RegistrationClass)
    {
        if (RegistrationClass->objectName().isEmpty())
            RegistrationClass->setObjectName(QStringLiteral("RegistrationClass"));
        RegistrationClass->resize(568, 418);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(RegistrationClass->sizePolicy().hasHeightForWidth());
        RegistrationClass->setSizePolicy(sizePolicy);
        RegistrationClass->setMinimumSize(QSize(568, 418));
        RegistrationClass->setMaximumSize(QSize(568, 418));
        verticalLayout_5 = new QVBoxLayout(RegistrationClass);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        groupBox = new QGroupBox(RegistrationClass);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setFlat(true);
        verticalLayout = new QVBoxLayout(groupBox);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setSizeConstraint(QLayout::SetFixedSize);
        label = new QLabel(groupBox);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout->addWidget(label);

        lineEdit = new QLineEdit(groupBox);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));

        horizontalLayout->addWidget(lineEdit);

        pushButton = new QPushButton(groupBox);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setAutoDefault(false);

        horizontalLayout->addWidget(pushButton);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_2->addWidget(label_2);

        lineEdit_2 = new QLineEdit(groupBox);
        lineEdit_2->setObjectName(QStringLiteral("lineEdit_2"));

        horizontalLayout_2->addWidget(lineEdit_2);

        pushButton_2 = new QPushButton(groupBox);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));
        pushButton_2->setAutoDefault(false);

        horizontalLayout_2->addWidget(pushButton_2);


        verticalLayout->addLayout(horizontalLayout_2);


        verticalLayout_5->addWidget(groupBox);

        groupBox_2 = new QGroupBox(RegistrationClass);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setFlat(true);
        verticalLayout_4 = new QVBoxLayout(groupBox_2);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        groupBox_3 = new QGroupBox(groupBox_2);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        horizontalLayout_6 = new QHBoxLayout(groupBox_3);
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        label_5 = new QLabel(groupBox_3);
        label_5->setObjectName(QStringLiteral("label_5"));

        horizontalLayout_5->addWidget(label_5);

        lineEdit_5 = new QLineEdit(groupBox_3);
        lineEdit_5->setObjectName(QStringLiteral("lineEdit_5"));

        horizontalLayout_5->addWidget(lineEdit_5);

        label_6 = new QLabel(groupBox_3);
        label_6->setObjectName(QStringLiteral("label_6"));

        horizontalLayout_5->addWidget(label_6);

        lineEdit_6 = new QLineEdit(groupBox_3);
        lineEdit_6->setObjectName(QStringLiteral("lineEdit_6"));

        horizontalLayout_5->addWidget(lineEdit_6);


        horizontalLayout_6->addLayout(horizontalLayout_5);


        verticalLayout_2->addWidget(groupBox_3);

        groupBox_4 = new QGroupBox(groupBox_2);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        horizontalLayout_8 = new QHBoxLayout(groupBox_4);
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        label_7 = new QLabel(groupBox_4);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setEnabled(true);

        horizontalLayout_7->addWidget(label_7);

        lineEdit_7 = new QLineEdit(groupBox_4);
        lineEdit_7->setObjectName(QStringLiteral("lineEdit_7"));
        lineEdit_7->setEnabled(true);

        horizontalLayout_7->addWidget(lineEdit_7);

        label_8 = new QLabel(groupBox_4);
        label_8->setObjectName(QStringLiteral("label_8"));

        horizontalLayout_7->addWidget(label_8);

        lineEdit_8 = new QLineEdit(groupBox_4);
        lineEdit_8->setObjectName(QStringLiteral("lineEdit_8"));

        horizontalLayout_7->addWidget(lineEdit_8);

        label_9 = new QLabel(groupBox_4);
        label_9->setObjectName(QStringLiteral("label_9"));

        horizontalLayout_7->addWidget(label_9);

        lineEdit_9 = new QLineEdit(groupBox_4);
        lineEdit_9->setObjectName(QStringLiteral("lineEdit_9"));

        horizontalLayout_7->addWidget(lineEdit_9);


        horizontalLayout_8->addLayout(horizontalLayout_7);


        verticalLayout_2->addWidget(groupBox_4);


        verticalLayout_4->addLayout(verticalLayout_2);


        verticalLayout_5->addWidget(groupBox_2);

        groupBox_5 = new QGroupBox(RegistrationClass);
        groupBox_5->setObjectName(QStringLiteral("groupBox_5"));
        groupBox_5->setFlat(true);
        groupBox_5->setCheckable(false);
        verticalLayout_3 = new QVBoxLayout(groupBox_5);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        label_3 = new QLabel(groupBox_5);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout_3->addWidget(label_3);

        lineEdit_10 = new QLineEdit(groupBox_5);
        lineEdit_10->setObjectName(QStringLiteral("lineEdit_10"));

        horizontalLayout_3->addWidget(lineEdit_10);

        pushButton_6 = new QPushButton(groupBox_5);
        pushButton_6->setObjectName(QStringLiteral("pushButton_6"));
        pushButton_6->setAutoDefault(false);

        horizontalLayout_3->addWidget(pushButton_6);


        verticalLayout_3->addLayout(horizontalLayout_3);

        pushButton_5 = new QPushButton(groupBox_5);
        pushButton_5->setObjectName(QStringLiteral("pushButton_5"));
        pushButton_5->setMinimumSize(QSize(93, 28));
        pushButton_5->setMaximumSize(QSize(93, 28));
        pushButton_5->setLayoutDirection(Qt::RightToLeft);
        pushButton_5->setAutoDefault(false);

        verticalLayout_3->addWidget(pushButton_5);


        verticalLayout_5->addWidget(groupBox_5);


        retranslateUi(RegistrationClass);

        pushButton_5->setDefault(true);


        QMetaObject::connectSlotsByName(RegistrationClass);
    } // setupUi

    void retranslateUi(QDialog *RegistrationClass)
    {
        RegistrationClass->setWindowTitle(QApplication::translate("RegistrationClass", "\347\202\271\344\272\221\351\205\215\345\207\206", 0));
        groupBox->setTitle(QApplication::translate("RegistrationClass", "\345\216\237\345\247\213\346\226\207\344\273\266\350\267\257\345\276\204", 0));
        label->setText(QApplication::translate("RegistrationClass", "\345\210\235\345\247\213\347\202\271\344\272\221\357\274\232", 0));
        pushButton->setText(QApplication::translate("RegistrationClass", "\351\200\211\346\213\251", 0));
        label_2->setText(QApplication::translate("RegistrationClass", "\347\233\256\346\240\207\347\202\271\344\272\221\357\274\232", 0));
        pushButton_2->setText(QApplication::translate("RegistrationClass", "\351\200\211\346\213\251", 0));
        groupBox_2->setTitle(QApplication::translate("RegistrationClass", "\345\217\202\346\225\260\350\256\276\347\275\256", 0));
        groupBox_3->setTitle(QApplication::translate("RegistrationClass", "Ransac\347\211\271\345\276\201\347\272\277\346\217\220\345\217\226", 0));
        label_5->setText(QApplication::translate("RegistrationClass", "\350\267\235\347\246\273\351\230\210\345\200\274\357\274\210m\357\274\211", 0));
        lineEdit_5->setText(QApplication::translate("RegistrationClass", "0.5", 0));
        label_6->setText(QApplication::translate("RegistrationClass", "\350\277\255\344\273\243\346\254\241\346\225\260", 0));
        lineEdit_6->setText(QApplication::translate("RegistrationClass", "30", 0));
        groupBox_4->setTitle(QApplication::translate("RegistrationClass", "ICP\351\205\215\345\207\206", 0));
        label_7->setText(QApplication::translate("RegistrationClass", "\350\277\255\344\273\243\346\254\241\346\225\260", 0));
        lineEdit_7->setText(QApplication::translate("RegistrationClass", "30", 0));
        label_8->setText(QApplication::translate("RegistrationClass", "\347\233\270\344\274\274\345\272\246\351\230\210\345\200\274", 0));
        lineEdit_8->setText(QApplication::translate("RegistrationClass", "0.2", 0));
        label_9->setText(QApplication::translate("RegistrationClass", "\350\267\235\347\246\273\351\230\210\345\200\274\346\214\207\346\225\260", 0));
        lineEdit_9->setText(QApplication::translate("RegistrationClass", "6", 0));
        groupBox_5->setTitle(QApplication::translate("RegistrationClass", "\350\276\223\345\207\272\347\202\271\344\272\221", 0));
        label_3->setText(QApplication::translate("RegistrationClass", "\347\202\271\344\272\221\344\277\235\345\255\230\350\267\257\345\276\204\357\274\232", 0));
        pushButton_6->setText(QApplication::translate("RegistrationClass", "\351\200\211\346\213\251", 0));
        pushButton_5->setText(QApplication::translate("RegistrationClass", "\345\274\200\345\247\213\351\205\215\345\207\206", 0));
    } // retranslateUi

};

namespace Ui {
    class RegistrationClass: public Ui_RegistrationClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_REGISTRATION_H
