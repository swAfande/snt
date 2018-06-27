#include "snt.h"

namespace snt
{


  IP NodeHandle::GetIP() const
  {
    return ip_;
  }

  bool NodeHandle::isNetwork(StatisticalNetwork* network) const
  {
    return network_ == network;
  }

  PingResult NodeHandle::Ping(const NodeAlias& alias)
  {
    EnsureValid();
    return PingResult(42);
  }

  void NodeHandle::RecieveData(const Data& data)
  {
    EnsureValid();
  }

  int NodeHandle::Socket(ESocketType type)
  {
    EnsureValid();

    return network_->CreateSocket(ip_, type);
  }

  int NodeHandle::Socket(int domain, ESocketType type, int protocol)
  {
    assert(false && "This function is not usable yet");
    EnsureValid();
    
    return 42;
  }

  bool NodeHandle::Connect(int sockfd, const NodeAlias& server)
  {
    EnsureValid();

    return network_->ConnectSocket(ip_, sockfd, server);
  }

  void NodeHandle::Close(int sockfd)
  {
    EnsureValid();

    network_->CloseSocket(ip_, sockfd);
  }

  void NodeHandle::Send(int sockfd, const Data& data)
  {
    EnsureValid();

    network_->SendData(ip_, sockfd, data);
  }

  void NodeHandle::Bind(int sockfd, const NodeAlias& address)
  {
    assert(false && "This function is not usable yet");
    EnsureValid();
  }

  void NodeHandle::BindAny(int sockfd)
  {
    EnsureValid();

    network_->BindSocketInaddrAny(ip_, sockfd);
  }

  void NodeHandle::Listen(int sockfd, int backlog)
  {
    EnsureValid();

    network_->SetListener(ip_, sockfd, backlog);
  }

  int NodeHandle::Accept(int sockfd, NodeAlias* sender /*= nullptr*/, bool nonblock /*= false*/)
  {
    EnsureValid();

    return network_->Accept(ip_, sockfd, sender, nonblock);
  }

  int NodeHandle::Recv(int sockfd, Data& data)
  {
    EnsureValid();

    return network_->RecieveMessageFromSocket(ip_, sockfd, data);
    return data.size();
  }

  NodeHandle::operator bool() const
  {
    return Initialized();
  }

  bool NodeHandle::Initialized() const
  {
    return network_ && ip_ != 0;
  }

  void NodeHandle::EnsureValid() const
  {
    if (!Initialized())
      throw exception::uninitialized_node();
  }

  void NodeHandle::CloseNode()
  {
    if (!Initialized())
      return;

    network_->CloseNode(ip_);
  }


}