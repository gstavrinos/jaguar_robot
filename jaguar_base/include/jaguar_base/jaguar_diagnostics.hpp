#ifndef JAGUAR_BASE_JAGUAR_DIAGNOSTICS_H
#define JAGUAR_BASE_JAGUAR_DIAGNOSTICS_H

#include "ros/ros.h"
#include "diagnostic_updater/diagnostic_updater.h"
//#include "jaguar_base/horizon_legacy_wrapper.h"
#include "jaguar_msgs/JaguarStatus.h"

namespace jaguar_base
{

  class JaguarSoftwareDiagnosticTask :
    public diagnostic_updater::DiagnosticTask
  {
  public:
    explicit JaguarSoftwareDiagnosticTask(jaguar_msgs::JaguarStatus &msg, double target_control_freq);

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

    void updateControlFrequency(double frequency);

  private:
    void reset();

    double control_freq_, target_control_freq_;
    jaguar_msgs::JaguarStatus &msg_;
  };

  template<typename T>
  class JaguarHardwareDiagnosticTask :
    public diagnostic_updater::DiagnosticTask
  {
  public:
    explicit JaguarHardwareDiagnosticTask(jaguar_msgs::JaguarStatus &msg);

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
      typename horizon_legacy::Channel<T>::Ptr latest = horizon_legacy::Channel<T>::requestData(1.0);
      if (latest)
      {
        update(stat, latest);
      }
    }

    void update(diagnostic_updater::DiagnosticStatusWrapper &stat, typename horizon_legacy::Channel<T>::Ptr &status);

  private:
    jaguar_msgs::JaguarStatus &msg_;
  };

  template<>
  JaguarHardwareDiagnosticTask<clearpath::DataSystemStatus>::JaguarHardwareDiagnosticTask(jaguar_msgs::JaguarStatus &msg);

  template<>
  JaguarHardwareDiagnosticTask<clearpath::DataPowerSystem>::JaguarHardwareDiagnosticTask(jaguar_msgs::JaguarStatus &msg);

  template<>
  JaguarHardwareDiagnosticTask<clearpath::DataSafetySystemStatus>::JaguarHardwareDiagnosticTask(
    jaguar_msgs::JaguarStatus &msg);

  template<>
  void JaguarHardwareDiagnosticTask<clearpath::DataSystemStatus>::update(
    diagnostic_updater::DiagnosticStatusWrapper &stat,
    horizon_legacy::Channel<clearpath::DataSystemStatus>::Ptr &status);

  template<>
  void JaguarHardwareDiagnosticTask<clearpath::DataPowerSystem>::update(
    diagnostic_updater::DiagnosticStatusWrapper &stat,
    horizon_legacy::Channel<clearpath::DataPowerSystem>::Ptr &status);

  template<>
  void JaguarHardwareDiagnosticTask<clearpath::DataSafetySystemStatus>::update(
    diagnostic_updater::DiagnosticStatusWrapper &stat,
    horizon_legacy::Channel<clearpath::DataSafetySystemStatus>::Ptr &status);

}
#endif
