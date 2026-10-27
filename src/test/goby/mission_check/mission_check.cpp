// Drives a closed-loop mission run of goby_helm_ivp and checks how it went.
//
// Once the helm reports PARK or DRIVE and the frontseat has reported nav,
// mission time starts. The checker
// publishes each configured poke (e.g. DEPLOY=true) as a Variable at its
// time, watches the helm's Postings for each expected key = value before its
// deadline, and fails at once if the helm is MALCONFIG or halts with a
// behavior error. After `duration` it exits 0 if every expectation was met
// (and, with drive_at_end, the helm is still in DRIVE), and 1 otherwise.

#include <goby/middleware/marshalling/protobuf.h>

#include <cmath>
#include <set>
#include <sstream>

#include <goby/middleware/frontseat/groups.h>
#include <goby/middleware/protobuf/frontseat_data.pb.h>
#include <goby/time/system_clock.h>
#include <goby/zeromq/application/single_thread.h>

#include "groups.h"
#include "interface/ivp.pb.h"
#include "mission_check_config.pb.h"

using goby::glog;

namespace ivpico
{
namespace goby
{
class MissionCheck : public ::goby::zeromq::SingleThreadApplication<protobuf::MissionCheckConfig>
{
  public:
    MissionCheck() : SingleThreadApplication(10 * boost::units::si::hertz)
    {
        interprocess().subscribe<groups::posting>([this](const ivp::interface::Posting& posting)
                                                  { handle(posting); });
        interprocess().subscribe<groups::status>([this](const ivp::interface::Status& status)
                                                 { handle(status); });
        interprocess().subscribe<::goby::middleware::frontseat::groups::node_status>(
            [this](const ::goby::middleware::frontseat::protobuf::NodeStatus&)
            {
                have_nav_ = true;
                maybe_start();
            });
    }

  private:
    double now() const { return ::goby::time::SystemClock::now<::goby::time::SITime>().value(); }

    void handle(const ivp::interface::Posting& posting)
    {
        if (start_ < 0)
            return;
        const std::string value = posting.value_case() == ivp::interface::Posting::kSval
                                      ? posting.sval()
                                      : number(posting.dval());
        for (int i = 0; i < cfg().expect_posting_size(); ++i)
        {
            const auto& expect = cfg().expect_posting(i);
            if (!met_.count(i) && expect.key() == posting.key() && same(expect.value(), value))
            {
                met_.insert(i);
                glog.is_verbose() && glog << "met " << expect.ShortDebugString() << " at "
                                          << now() - start_ << " s" << std::endl;
            }
        }
    }

    void handle(const ivp::interface::Status& status)
    {
        state_ = status.state();
        maybe_start();
        if (state_ == ivp::interface::Status::MALCONFIG)
            finish(false, "helm is MALCONFIG");
        else if (status.allstop() == ivp::interface::BEHAVIOR_ERROR)
            finish(false, "behavior error: " + status.halt_message());
    }

    void maybe_start()
    {
        if (start_ < 0 && have_nav_ &&
            (state_ == ivp::interface::Status::PARK || state_ == ivp::interface::Status::DRIVE))
            start_ = now();
    }

    void loop() override
    {
        if (start_ < 0)
            return;
        const double t = now() - start_;
        for (int i = 0; i < cfg().poke_size(); ++i)
        {
            const auto& poke = cfg().poke(i);
            if (!poked_.count(i) && t >= poke.at())
            {
                ivp::interface::Variable variable;
                variable.set_key(poke.key());
                variable.set_sval(poke.value());
                interprocess().publish<groups::variable>(variable);
                poked_.insert(i);
            }
        }
        for (int i = 0; i < cfg().expect_posting_size(); ++i)
        {
            const auto& expect = cfg().expect_posting(i);
            if (!met_.count(i) && t > expect.before())
                finish(false, "no " + expect.key() + " = " + expect.value() + " by " +
                                  number(expect.before()) + " s");
        }
        if (t >= cfg().duration())
        {
            if (cfg().drive_at_end() && state_ != ivp::interface::Status::DRIVE)
                finish(false, "helm not in DRIVE at the end");
            else
                finish(true, "");
        }
    }

    void finish(bool ok, const std::string& why)
    {
        if (done_)
            return;
        done_ = true;
        std::cout << (ok ? "PASSED" : "FAILED: " + why) << std::endl;
        quit(ok ? 0 : 1);
    }

    static std::string number(double d)
    {
        std::ostringstream os;
        os << d;
        return os.str();
    }

    static bool same(const std::string& expected, const std::string& value)
    {
        char* end_a = nullptr;
        char* end_b = nullptr;
        const double a = std::strtod(expected.c_str(), &end_a);
        const double b = std::strtod(value.c_str(), &end_b);
        if (*end_a == '\0' && *end_b == '\0' && !expected.empty() && !value.empty())
            return std::fabs(a - b) < 1e-6;
        return expected == value;
    }

    double start_{-1};
    ivp::interface::Status::State state_{ivp::interface::Status::STATE_UNKNOWN};
    std::set<int> poked_, met_;
    bool have_nav_{false};
    bool done_{false};
};
} // namespace goby
} // namespace ivpico

int main(int argc, char* argv[]) { return goby::run<ivpico::goby::MissionCheck>(argc, argv); }
