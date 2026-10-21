#pragma once

// Included by hl_command.hpp
#include <functional>
#include <iostream>
#include <utility>

template <typename TRobotInterface, typename TData, typename... TArgs>
HLCommand<TRobotInterface, TData, TArgs...>::HLCommand(std::string name, std::string doc)
    : name_(std::move(name))
    , doc_(std::move(doc))
{}

template <typename TRobotInterface, typename TData, typename... TArgs>
void HLCommand<TRobotInterface, TData, TArgs...>::registerCommand(dls::CommandManager& command_manager)
{
    command_manager.addCommand<TArgs...>
    (
        name_,
        doc_,
        std::function<bool(TArgs...)>([this](TArgs... args) -> bool { return receive(args...); }),
        {},
        true
    );
}

template <typename TRobotInterface, typename TData, typename... TArgs>
bool HLCommand<TRobotInterface, TData, TArgs...>::receive(TArgs... args)
{
    TData data{};
    if (!parse(args..., data))
        return false;
    if (!queue_.push(data)) {
        std::cerr << name_ << ": command queue full, command dropped" << std::endl;
        return false;
    }
    return true;
}

template <typename TRobotInterface, typename TData, typename... TArgs>
void HLCommand<TRobotInterface, TData, TArgs...>::applyReceived(GenericRobotInterface& source)
{
    // FastDDSHLInterface::addCommand() only accepts commands whose TRobotInterface
    // is a base of its robot interface, so source is a TRobotInterface.
    auto& robot_interface = static_cast<TRobotInterface&>(source);
    TData data;
    while (queue_.pop(data))
        apply(data, robot_interface);
}
