#include <iostream>
#include <cstdlib>

#include "snt.h"
#include "examples.h"

int main()
{
  using namespace snt;

  std::cout << "Hello world!\n";

  Emulator emu;
  StatisticalNetwork network = emu.InitializeNetwork(1000000, NetworkModel::eErdeshRenyi);
  NodeHandle tima_node = network.ProvideNode("tima");
  NodeHandle dima_node = network.ProvideNode("dima");

  network.ExpectedDistance(tima_node, dima_node, 1000);

  std::cout << "tima_node.Ping(\"dima\"): " << tima_node.Ping("dima").ms_ << "ms\n";

  RunEchoExample();


  return EXIT_SUCCESS;
}
