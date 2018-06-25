#include <iostream>
#include <map>
#include <cstring>
#include <future>
#include <atomic>

#include "snt.h"
//#include "chat_message.hpp"

using namespace snt;

/*class chat_client
{
public:
  chat_client() {}

  chat_client(NodeHandle node, NodeAlias server_address) : 
      node_(node), server_address_(server_address), 
      socket_(node.socket(ESocketType::eStream))
  {    
    node_.connect(socket_, server_address_);
  }

  ~chat_client() 
  {
    node_.close(socket_);
  }  

  void write(const chat_message& msg)
  {
    Data data(msg.length());
    memcpy(data.data(), msg.data(), data.size());

    node_.send(socket_, data);
  }

private:
  NodeHandle node_;
  NodeAlias server_address_;
  int socket_;
}

void ChatExapmple()
{
  Emulator emu;
  StatisticalNetwork network = 
    emu.InitializeNetwork(1000000, NetworkModel::eErdeshRenyi);

  NodeAlias server_address = "server_yima";

  NodeAlias dima_address = "user_dima";
  NodeAlias tima_address = "user_tima";

  NodeHandle tima_node = network.ProvideNode(tima_address);
  NodeHandle dima_node = network.ProvideNode(dima_address);
  NodeHandle yima_node = network.ProvideNode(server_address);

  std::map<NodeHandle, chat_client> clients;
  clients[tima_node] = chat_client(tima_node, server_address);
  clients[dima_node] = chat_client(dima_node, server_address);


  char line[chat_message::max_body_length + 1] = "";
    
  NodeHandle active_node;
  while (std::cin.getline(line, chat_message::max_body_length + 1))
  {
    if (line[0] == ':')
    {
      if (tima_address == line + 1)
        active_node = tima_node;
      else if (dima_address == line + 1)
        active_node = dima_node;
      else
        std::cout << "no user \"" << (line + 1) << "\"" << std::endl;

      continue;
    }

    if (!active_node)
    {
      std::cout << "Select a user. Usage: \":user_address\"" << std::endl;
      continue;
    }

    chat_message msg;
    msg.body_length(std::strlen(line));
    std::memcpy(msg.body(), line, msg.body_length());
    msg.encode_header();

    clients[active_node].write(msg);

    //c.write(msg);
  }
}*/

void ServerPart(NodeHandle server, const std::atomic<bool>* done)
{
  if (!server || !done)
    return;

  int listener = server.Socket(ESocketType::eStream);
  if (listener < 0)
    return;

  server.BindAny(listener);
  server.Listen(listener, 1);

  Data buffer(1024, 0);
  while (!done->load())
  {
    int sock = server.Accept(listener);
    if (sock < 0)
      return;

    while (1)
    {
      int bytes_read = server.Recv(sock, buffer);
      if (bytes_read == -1)
        break;
      server.Send(sock, Data(buffer.begin(), buffer.begin() + bytes_read));
    }
    server.Close(sock);
  }
}

void ClientPart(NodeHandle client, const std::string& msg)
{
  if (!client)
    return;

  int sock = client.Socket(ESocketType::eStream);
  if (sock < 0)
    return;

  if (!client.Connect(sock, "server"))
    return;

  {
    Data data(msg.length() + 1, 0);
    memcpy(data.data(), msg.data(), msg.length());
    client.Send(sock, data);
  }

  {
    Data buffer(msg.length() + 1, 0);
    client.Recv(sock, buffer);
    std::cout << "Client: " << 
      reinterpret_cast<const char*>(buffer.data()) << std::endl;
  }
}

void RunEchoExample()
{
  Emulator emu;
  StatisticalNetwork network = 
    emu.InitializeNetwork(1000000, NetworkModel::eErdeshRenyi);

  NodeHandle client = network.ProvideNode("client");
  NodeHandle server = network.ProvideNode("server");


  std::atomic<bool> my_job_here_is_done(false);

  std::thread server_thread(ServerPart, server, &my_job_here_is_done);

  std::thread client_thread(ClientPart, client, 
                            "The lady doth protest too much, methinks.");

  client_thread.join();
  client.CloseNode();

  my_job_here_is_done.store(true);
  server_thread.join();
}