#ifndef _SNT_H_
#define _SNT_H_

#include <string>
#include <vector>
#include <cstdint>
#include <set>
#include <map>
#include <utility>
#include <cassert>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <memory>
#include <atomic>
#include <future>
#include <random>

namespace snt
{
  namespace exception
  {
    struct uninitialized_node {};
  }

  namespace utility
  {
    template<typename T>
    class IndexProvider
    {
    public:
      explicit IndexProvider(T initial_value = 0) : 
        next_new_index_(initial_value) {}
      ~IndexProvider() {}

      void FreeIndex(T index)
      {
        if (index < next_new_index_)
          reusable_.insert(index);
      }

      T CreateIndex()
      {
        if (reusable_.empty())
          return next_new_index_++;

        T index = *reusable_.begin();
        reusable_.erase(reusable_.begin());

        return index;
      }

    private:
      T next_new_index_;
      std::set<T> reusable_;
    };
  }

  enum class NetworkModel
  {
    eErdeshRenyi = 0
  };

  enum class ESocketType
  {
    eStream,
    eDatagram,
    eRaw
  };

  enum class ESocketState
  {
    eNormal,
    eListener
  };

  using NodeAlias = std::string;
  using Data = std::vector<std::uint8_t>;
  using IP = std::uintmax_t;

  class StatisticalNetwork;


  struct PingResult
  {
    PingResult(unsigned int ms) : ms_(ms) {}

    unsigned int ms_ = -1;
  };

  class NodeHandle
  {
  public:
    NodeHandle(StatisticalNetwork* network = nullptr, IP ip = 0) :
      network_(network), ip_(ip) {}

    IP GetIP() const;
    bool isNetwork(StatisticalNetwork* network) const;

    void CloseNode();

    void RecieveData(const Data& data);

    PingResult Ping(const NodeAlias& alias);

    int Socket(int domain, ESocketType type, int protocol);
    int Socket(ESocketType type);
    bool Connect(int sockfd, const NodeAlias& server);
    void Close(int sockfd);
    void Send(int sockfd, const Data& data);
    void Bind(int sockfd, const NodeAlias& address);
    void BindAny(int sockfd);
    void Listen(int sockfd, int backlog);
    int Accept(int sockfd, NodeAlias* sender = nullptr, bool nonblock = false);
    int Recv(int sockfd, Data& data);

    operator bool() const;

  private:
    bool Initialized() const;
    void EnsureValid() const;    

    StatisticalNetwork* network_;
    IP ip_;
  };

  class PathHandle
  {
  public:
    PathHandle() {}

    bool Exist() const
    {
      return true;
    }
  };

  struct ConnectionRequest
  {
    IP from_;
    int socket_;
    std::shared_ptr<bool> accepted_;
  };

  struct Connection
  {
    IP ip1_;
    int socket1_;
    IP ip2_;
    int socket2_;

    PathHandle path_;
  };

  struct SocketDescription
  {
    SocketDescription() : isClosing(false) {}

    ESocketType type_ = ESocketType::eStream;
    ESocketState state_ = ESocketState::eNormal;
    IP address_;
    PathHandle path_;
    int connection_ = -1;

    int backlog_ = 0;
    std::queue<ConnectionRequest> requests_;
    std::mutex requests_mtx_;
    std::condition_variable requests_cv_;

    Data message_;
    std::mutex message_mtx_;
    std::condition_variable message_cv_;

    std::atomic<bool> isClosing;
    //bool isClosing = false;

    SocketDescription(const SocketDescription&) = delete;
    SocketDescription& operator=(const SocketDescription&) = delete;
  };

  // статистикал, потому что её работа определяется вероятностными параметрами
  class StatisticalNetwork
  {
  public:
    using dist_dype = double;

    StatisticalNetwork() : node_ip_provider_(1) {}
    StatisticalNetwork(StatisticalNetwork&& aFrom) :
      named_nodes_(std::forward<StatisticalNetwork>(aFrom).named_nodes_),
      node_ip_provider_(std::forward<StatisticalNetwork>(aFrom).node_ip_provider_),
      sockets_(std::forward<StatisticalNetwork>(aFrom).sockets_),
      socket_index_provider_(std::forward<StatisticalNetwork>(aFrom).socket_index_provider_),
      binded_sockets_(std::forward<StatisticalNetwork>(aFrom).binded_sockets_),
      connection_(std::forward<StatisticalNetwork>(aFrom).connection_),
      connection_index_provider_(std::forward<StatisticalNetwork>(aFrom).connection_index_provider_),
      listeners_(std::forward<StatisticalNetwork>(aFrom).listeners_)
    {

    }

    NodeHandle ProvideNode(const NodeAlias& alias)
    {
      auto it = named_nodes_.find(alias);
      if (it != named_nodes_.end())
        return NodeHandle(this, it->second);

      IP ip = GrantNewIP();
      named_nodes_[alias] = ip;

      return NodeHandle(this, ip);
    }

    /*void SetTransmissionQualityDistribution(void)
    {
      // Каким-то образом задаём вероятность что прямое соединение между 
      // двумя нодами будет иметь такое-то качество (double, >0).
      // Что оно определяет - см. ExpectedDistance
    }*/

    void ExpectedDistance(
      NodeHandle node1, NodeHandle node2, dist_dype distance)
    {
      // Задаём расстояние между двумя нодами

      // distance - некоторая величина, характеризующая расстояние между 
      // двумя нодами. Если, например, между ними есть прямое соедиенение, 
      // то время, за которое сигнал проходит между ними, равно quality * distance, 
      // ну или среднее время, на что фантазии хватит
    }

    PathHandle FindPath(NodeHandle from, NodeHandle to)
    {
      // Ищем(а на самом деле генерируем, т.к мы в случайном графе) 
      // путь от одного нода к другому. Пофиг если мы уже нашли один,
      // для выбора уже найденного другая функция: см. GetPath

      // Смысл в том, что по одному пути данные идут примерно одинаково при
      // каждой передаче

      return PathHandle();
    }

    PathHandle GetPath(NodeHandle from, NodeHandle to)
    {
      // Если нет известных путей, находим через FindPath,
      // иначе выбираем лучший

      if (!from || !from.isNetwork(this) || !to || !to.isNetwork(this))
        return PathHandle();

      return GetPath(from.GetIP(), to.GetIP());
    }


    void SendData(NodeHandle from, NodeHandle to, const Data& data)
    {
      // Ну понятно, из from в to отправляем какие-то данные

      // Но(!) они идут некоторое время. Как это устроить? Ну для начала можно 
      // просто возвращать, сколько прошло. Время для данных какого-то 
      // ненулевого объёма зависит уже от пропускной способности (100мб/с),
      // А для маленьких, типа когда мы пингуем, от длины пути. 
      // Как их(пропускные способности) задавать, можешь сам придумать, 
      // можно забить и сделать везде одинаковую.

      // Данные посылаются по конкретному(!) пути (GetPath/FindPath)

      // После того, как данные прошли по сетке до пункта назначения, вызываем
      // функцию получения данных у to:
      to.RecieveData(data);

      // (*) Если интересно, можно добавлять шум в данные
      // (*) Вообще для полноценной симуляции должна быть уже какая-то 
      //     полноценная временнАя модель(в реальном времени - тогда придётся
      //     всякую многопоточность мутить, или дискретную - тогда придётся
      //     вводить какую-то очередь выполнения, и шаги времени)
      // (*) Ещё можно добавить смертность нодов или связей между ними, т.е.
      //     иногда они умирают! И, если на известном пути между нодами, 
      //     что-то умерло, надо искать новый путь!
    }

    void SendData(PathHandle path, const Data& data)
    {
      // Оно же, но с явным указанием пути
    }

    //////////////////////////////////////////////////////////////////////////
    // Интерфейс для NodeHandle

    const SocketDescription* GetSocketDescription(IP ip, int sockfd) const
    {
      auto map_key = std::make_pair(ip, sockfd);
      if (!sockets_.count(map_key))
        return nullptr;

      auto it = sockets_.find(std::make_pair(ip, sockfd));
      if (it == sockets_.end())
        return nullptr;

      return &it->second;
    }

    int CreateSocket(IP ip, ESocketType type)
    {
      int sock = socket_index_provider_.CreateIndex();
      sockets_[std::make_pair(ip, sock)].type_ = type;
      sockets_for_node_[ip].insert(sock);

      return sock;
    }

    bool ConnectSocket(IP ip, int sockfd, const NodeAlias& address)
    {
      if (!DoesSocketExist(ip, sockfd))
        return false;

      if (!DoesAliasExist(address))
        return false;

      SocketDescription& description = sockets_[std::make_pair(ip, sockfd)];
      description.address_ = named_nodes_[address];

      if (description.type_ == ESocketType::eStream)
      {
        description.path_ = GetPath(ip, named_nodes_[address]);

        if (!description.path_.Exist())
          return false;

        int listener = -1;
        {
          std::unique_lock<std::mutex> lck(listeners_mutex_);
          while ((listener = GetListener(description.address_)) == -1)
            listeners_cv_.wait(lck);
        }


        ConnectionRequest request;
        request.from_ = ip;
        request.socket_ = sockfd;
        request.accepted_ = std::make_shared<bool>(false);

        if (!QueueConnectionRequest(description.address_, listener, request))
          return false;

        std::unique_lock<std::mutex> lck(request_mutex_);
        while (request.accepted_ && !*request.accepted_)
          request_cv_.wait(lck);
      }

      return true;
    }

    void CloseSocket(IP ip, int sockfd)
    {
      if (!DoesSocketExist(ip, sockfd))
        return;

      /*SocketDescription socket_description = GetSocketDescription(ip, sockfd);

      if (socket_description.type_ == ESocketType::eStream)
      {
        SendData(socket_description.path_, Data());
      }
      else
      {
        if (DoesAliasExist(socket_description.address_))
          SendData(ip, named_nodes_[socket_description.address_], Data());
      }
      */

      const SocketDescription* description = GetSocketDescription(ip, sockfd);
      if (description && description->type_ == ESocketType::eStream)
        CloseConnection(description->connection_);

      SendData(ip, sockfd, Data());

      /*sockets_.erase(sockets_.find(std::make_pair(ip, sockfd)));
      socket_index_provider_.FreeIndex(sockfd);*/
    }

    void SendData(IP ip, int sockfd, const Data& data)
    {
      if (!DoesSocketExist(ip, sockfd))
        return;

      const SocketDescription* socket_description = GetSocketDescription(ip, sockfd);

      if (!socket_description)
        return;

      if (socket_description->type_ == ESocketType::eStream)
      {
        //SendData(socket_description->path_, data);
        Connection connection = connection_[socket_description->connection_];
        IP target_ip = 0;
        int target_socket = 0;

        if (connection.ip1_ == ip && connection.socket1_ == sockfd)
        {
          target_ip = connection.ip2_;
          target_socket = connection.socket2_;
        }
        else if (connection.ip2_ == ip && connection.socket2_ == sockfd)
        {
          target_ip = connection.ip1_;
          target_socket = connection.socket1_;
        }
        else
        {
          return;
        }

        if (!DoesSocketExist(target_ip, target_socket))
          return;

        SocketDescription& target_description = 
          sockets_[std::make_pair(target_ip, target_socket)];

        SocketDescription* pDescription = &target_description;

        if (target_description.isClosing.load())
          return;

        std::chrono::milliseconds transmission_time = 
          GetTimeForConnection(socket_description->connection_);

        std::async([pDescription, data, transmission_time]()
        {
          std::unique_lock<std::mutex> lck(pDescription->message_mtx_);
          while (!pDescription->message_.empty() && !pDescription->isClosing.load())
            pDescription->message_cv_.wait(lck);

          if (pDescription->isClosing.load())
            return;

          std::this_thread::sleep_for(transmission_time);

          pDescription->message_ = data;
          pDescription->message_cv_.notify_all();
        });

        //target_description.
      }
      else
      {
        /*if (DoesAliasExist(socket_description.address_))
          SendData(ip, named_nodes_[socket_description.address_], data);*/
        SendData(ip, socket_description->address_, data);
      }
    }

    void BindSocketInaddrAny(IP ip, int sockfd)
    {
      if (!DoesSocketExist(ip, sockfd))
        return;

      binded_sockets_[ip].insert(sockfd);
    }

    void SetListener(IP ip, int sockfd, int backlog)
    {
      if (!DoesSocketExist(ip, sockfd))
        return;

      SocketDescription& description = sockets_[std::make_pair(ip, sockfd)];
      description.state_ = ESocketState::eListener;
      description.backlog_ = backlog;

      listeners_[ip].insert(sockfd);
    }

    int Accept(IP ip, int sockfd, NodeAlias* sender, bool nonblock = false)
    {
      if (!DoesSocketExist(ip, sockfd))
        return -1;

      SocketDescription& description = sockets_[std::make_pair(ip, sockfd)];

      if (description.state_ != ESocketState::eListener)
        return -1;      

      ConnectionRequest request;
      {
        std::unique_lock<std::mutex> lck(description.requests_mtx_);
        if (nonblock)
        {
          if (description.requests_.empty() || description.isClosing.load())
            return -1;
        }
        else
        {
          while (description.requests_.empty() && !description.isClosing.load())
            description.requests_cv_.wait(lck);

          if (description.isClosing.load())
            return -1;
        }

        assert(!description.requests_.empty());
        request = description.requests_.front();
        description.requests_.pop();
      }

      int sock = CreateSocket(ip, ESocketType::eStream);
      if (sock == -1)
        return -1;

      assert(DoesSocketExist(ip, sock));

      if (!DoesSocketExist(request.from_, request.socket_))
        return -1;

      sockets_[std::make_pair(ip, sock)].address_ = request.from_;
      sockets_[std::make_pair(ip, sock)].connection_ = MakeConnection(
        ip, sock, request.from_, request.socket_);
      sockets_[std::make_pair(request.from_, request.socket_)].connection_ =
        sockets_[std::make_pair(ip, sock)].connection_;

      {
        std::unique_lock<std::mutex> lck(request_mutex_);
        if (request.accepted_)
          *request.accepted_ = true;
        request_cv_.notify_all();
      }

      return sock;
    }

    int RecieveMessageFromSocket(IP ip, int sockfd, Data& data)
    {
      if (!DoesSocketExist(ip, sockfd))
        return -1;

      SocketDescription& description = sockets_[std::make_pair(ip, sockfd)];

      std::unique_lock<std::mutex> lck(description.message_mtx_);
      while (description.message_.empty() && !description.isClosing.load())
        description.message_cv_.wait(lck);

      if (description.isClosing.load())
        return -1;

      data = Data();
      std::swap(data, description.message_);
      
      return data.size();
    }

    void CloseNode(IP ip)
    {
      for (int socket : sockets_for_node_[ip])
        CloseSocket(ip, socket);
    }    

  private:    
    IP GrantNewIP()
    {
      return node_ip_provider_.CreateIndex();
    }

    bool DoesSocketExist(IP ip, int sockfd) const
    {
      return sockets_.count(std::make_pair(ip, sockfd)) > 0;
    }

    bool DoesAliasExist(const NodeAlias& alias) const
    {
      return named_nodes_.count(alias) > 0;
    }

    PathHandle GetPath(IP from, IP to)
    {
      return PathHandle();
    }

    void SendData(IP from, IP to, const Data& data)
    {

    }

    int MakeConnection(IP ip1, int socket1, IP ip2, int socket2)
    {
      int index = connection_index_provider_.CreateIndex();

      connection_[index].ip1_ = ip1;
      connection_[index].socket1_ = socket1;
      connection_[index].ip2_ = ip2;
      connection_[index].socket2_ = socket2;
      connection_[index].path_ = GetPath(ip1, ip2);

      return index;
    }

    int GetListener(IP ip) const
    {
      auto it = listeners_.find(ip);
      if (it == listeners_.end() || it->second.empty())
        return false;

      return *it->second.begin();
    }

    bool QueueConnectionRequest(
        IP ip, int sockfd, const ConnectionRequest& request)
    {
      if (!DoesSocketExist(ip, sockfd))
        return false;

      SocketDescription& description = sockets_[std::make_pair(ip, sockfd)];
      if (description.state_ != ESocketState::eListener)
        return false;

      std::unique_lock<std::mutex> lck(description.requests_mtx_);
      description.requests_.push(request);
      description.requests_cv_.notify_all();

      return true;
    }

    void CloseConnection(int connection)
    {
      if (connection == -1)
        return;

      Connection connection_struct = connection_[connection];
      if (DoesSocketExist(connection_struct.ip1_, connection_struct.socket1_))
      {
        SocketDescription& description = sockets_[
          std::make_pair(connection_struct.ip1_, connection_struct.socket1_)];

        description.isClosing.store(true);
        description.message_cv_.notify_all();
        description.requests_cv_.notify_all();
      }

      if (DoesSocketExist(connection_struct.ip2_, connection_struct.socket2_))
      {
        SocketDescription& description = sockets_[
          std::make_pair(connection_struct.ip2_, connection_struct.socket2_)];

        description.isClosing.store(true);
        description.message_cv_.notify_all();
        description.requests_cv_.notify_all();
      }
    }

    std::chrono::milliseconds GetTimeForConnection(int connection)
    {
      std::default_random_engine generator;
      std::normal_distribution<double> distribution(42, 5);

      double time = distribution(generator);
      if (time < 0)
        time = 0;

      return std::chrono::milliseconds(static_cast<int>(time));
    }

    std::map<NodeAlias, IP> named_nodes_;
    utility::IndexProvider<IP> node_ip_provider_;

    std::map<std::pair<IP, int>, SocketDescription> sockets_;
    utility::IndexProvider<int> socket_index_provider_;

    std::map<IP, std::set<int>> binded_sockets_;

    std::map<int, Connection> connection_;
    utility::IndexProvider<int> connection_index_provider_;

    std::map<IP, std::set<int>> listeners_;
    std::mutex listeners_mutex_;
    std::condition_variable listeners_cv_;

    std::mutex request_mutex_;
    std::condition_variable request_cv_;

    std::map<IP, std::set<int>> sockets_for_node_;

    StatisticalNetwork(const StatisticalNetwork&) = delete;
    StatisticalNetwork& operator=(const StatisticalNetwork&) = delete;

  };


  class Emulator
  {
  public:
    Emulator() {}

    StatisticalNetwork InitializeNetwork(
      int number_of_nodes, NetworkModel model)
    {
      // Ну тут собсна инициализируем нашу сетку. Пока можно ограничиться
      // эрдёшем-реньи G(n,p): между двумя(из n) узлами есть
      // путь(в обе стороны) с вероятостью p
      // Но можешь посмотреть чё-нить ещё, типа боллобаша-риордана (см.
      // Райгородский, модели случайных графов") или своё чё

      return StatisticalNetwork();
    }

  };
}

#endif // !_SNT_H_
