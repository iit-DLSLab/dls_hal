#pragma once

#include "dls_hal/generic_robot_interface.hpp"
#include "dls2/command/command_manager.hpp"

#include <boost/lockfree/queue.hpp>
#include <string>
#include <type_traits>

// Non-templated interface of the console commands received by the high-level interface,
// so that commands of any type are stored together and applied in a single loop.
class HLCommandBase {
public:
    virtual ~HLCommandBase() = default;

    // Registers the command on command_manager. Called once, outside the real-time thread.
    virtual void registerCommand(dls::CommandManager& command_manager) = 0;
    // Applies the commands received since the last call to source, in arrival order.
    // Non-blocking and allocation free: called by the real-time thread.
    virtual void applyReceived(GenericRobotInterface& source) = 0;
};

// Console command taking the arguments TArgs... and applied to a TRobotInterface.
// The DDS thread receiving the command converts its arguments into a TData request with
// parse() and queues it; the real-time thread pops the requests and calls apply().
// TData is copied through a lock-free queue, so it must be trivially copyable.
template <typename TRobotInterface, typename TData, typename... TArgs>
class HLCommand : public HLCommandBase {
    static_assert(std::is_base_of_v<GenericRobotInterface, TRobotInterface>,
                  "HLCommand: TRobotInterface must derive from GenericRobotInterface");
    static_assert(std::is_trivially_copyable_v<TData> && std::is_trivially_destructible_v<TData>,
                  "HLCommand: TData must be trivially copyable and destructible");
public:
    using RobotInterface = TRobotInterface;

    HLCommand(std::string name, std::string doc);

    void registerCommand(dls::CommandManager& command_manager) final;
    void applyReceived(GenericRobotInterface& source) final;

    const std::string& name() const { return name_; }

protected:
    // Validates the console arguments and converts them into data. Runs on the DDS thread
    // receiving the command, so it may print and allocate. Returns false to reject the command.
    virtual bool parse(TArgs... args, TData& data) = 0;
    // Applies data to source. Runs on the real-time thread: must not block or allocate.
    virtual void apply(const TData& data, TRobotInterface& source) = 0;

private:
    // Called by the DDS thread receiving the command.
    bool receive(TArgs... args);

    std::string name_;
    std::string doc_;
    // Lock-free multi-producer queue: different DDS threads may deliver the command.
    boost::lockfree::queue<TData, boost::lockfree::capacity<64>> queue_;
};

#include "dls_hal/hl_command.tpp"
