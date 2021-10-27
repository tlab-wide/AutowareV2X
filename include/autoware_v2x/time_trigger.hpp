#ifndef TIME_TRIGGER_HPP_XRPGDYXO
#define TIME_TRIGGER_HPP_XRPGDYXO

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <vanetza/common/manual_runtime.hpp>

class TimeTrigger
{
public:
  TimeTrigger(boost::asio::io_service &);
  vanetza::Runtime & runtime() { return runtime_; }
  void schedule();

private:
  boost::posix_time::ptime now() const;
  void on_timeout(const boost::system::error_code &);
  void update_runtime();

  boost::asio::io_service & io_service_;
  boost::asio::deadline_timer timer_;
  vanetza::ManualRuntime runtime_;
};

#endif /* TIME_TRIGGER_HPP_XRPGDYXO */
