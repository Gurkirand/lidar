#include <iostream>
#include <string>

#include "../include/vlpmap.h"
#include <pcl/console/parse.h>

int main(int argc, char* argv[])
{
    if( pcl::console::find_switch( argc, argv, "-help" ) ){
        std::cout << "usage: " << argv[0]
            << " [-ipaddress <192.168.1.70>]"
            << " [-port <2368>]"
            << " [-pcap <*.pcap>]"
            << " [-help]"
            << std::endl;
        return 0;
    }

    /* std::string ipaddress( "192.168.1.70" ); */
    /* std::string port( "2368" ); */
    std::string pcap;

    /* pcl::console::parse_argument( argc, argv, "-ipaddress", ipaddress ); */
    /* pcl::console::parse_argument( argc, argv, "-port", port ); */
    pcl::console::parse_argument( argc, argv, "-pcap", pcap );

    /* std::cout << "-ipadress : " << ipaddress << std::endl; */
    /* std::cout << "-port : " << port << std::endl; */
    std::cout << "-pcap : " << pcap << std::endl;

	velodyne::vlpmap vlpmap(pcap);
	vlpmap.run();

	return 0;
}

