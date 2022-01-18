/*!
MIT License

Copyright (c) 2022 Gotelli Andrea

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/



#ifndef PARAMETERS_VECTOR_SAVING_H
#define PARAMETERS_VECTOR_SAVING_H

#include <QWidget>

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>
#include <QtWidgets/QPushButton>

#include <QFile>
#include <QtWidgets/QMessageBox>

#include <yaml-cpp/yaml.h>
#include <fstream>

class ParametersVectorSaving : public QWidget
{
    Q_OBJECT
public:
    explicit ParametersVectorSaving(const QString &file_name_,
                                    const std::vector<float> &parameters_vector_,
                                    QWidget *parent_widget = nullptr);

signals:

  void Saved();


private:
    QGridLayout *gridLayout;
    QGridLayout *parameters_vector_gridLayout;
    QLabel *parameters_vector_name_label;
    QLineEdit *parameters_vector_name_lineEdit;
    QGridLayout *brief_gridLayout;
    QLabel *brief_label;
    QTextEdit *brief_textEdit;
    QSpacerItem *brief_verticalSpacer;
    QDialogButtonBox *buttonBox;

    const QString &file_name;
    const std::vector<float> parameters_vector;

    void SaveParametersVector(const QString &section_name, const QString &brief);

    void AddSection(YAML::Emitter &emitter, const std::string &section_name,
                    const std::vector<float> &parameters, const std::string &brief);

};

#endif // PARAMETERS_VECTOR_SAVING_H
