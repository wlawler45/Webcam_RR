
#include "Webcam_RR.h"

int main(int argc, char* argv[])
{

	std::string dummy;
	
	
	ServerNodeSetup node_setup(ROBOTRACONTEUR_SERVICE_TYPES,"opencv.webcams",8888);


	if (argc == 2 && argv[1] == "List") {


	}
	std::vector<webcaminputs> indexes;
	
	if (argc == 1) {
		//indexes.push_back(0);
		std::cout << "Attempting to start default webcam, to see list of available webcams give command line argument List" << std::endl;
	}
	
	if (argc > 1) {
		for (int i = 1; i < argc; i++) {
			if (strcmp(argv[i], "--camera-id")) {
				struct webcaminputs argin;
				argin.is_index=true;
				argin.has_fps = false;
				argin.has_resolution = false;
				argin.index=atoi(argv[i+1]);
				for (int j = 0; j < sizeof(argv[i + 2]); j++) {
					if (argv[i + 2][j] =='x') {
						char* part;
						memcpy(part, 0, j - 1);
						argin.width = atoi(part);
						char* part2;
						int lengthleft = sizeof(argv[i + 2]) - j;
						memcpy(part2, j, lengthleft);
						argin.height = atoi(part2);
						argin.has_resolution = true;
					}
				}
				if (std::strstr(argv[i + 3], "fps")) {
					char* part3;
					memcpy(part3, 0, sizeof(argv[i + 3]) - 3);
					argin.fps = atoi(part3);
					argin.has_fps = true;
				}
				
				indexes.push_back(argin);
			}
			if (strcmp(argv[i], "--camera-name")) {
				struct webcaminputs argin;
				argin.is_index = false;
				argin.has_fps = false;
				argin.has_resolution = false;
				argin.camera_name = std::string(argv[i + 1]);
				for (int j = 0; j < sizeof(argv[i + 2]); j++) {
					if (argv[i + 2][j] == 'x') {
						char* part;
						memcpy(part, 0, j - 1);
						argin.width = atoi(part);
						char* part2;
						int lengthleft = sizeof(argv[i + 2]) - j;
						memcpy(part2, j, lengthleft);
						argin.height = atoi(part2);
						argin.has_resolution = true;
					}
				}
				if (std::strstr(argv[i + 3], "fps")) {
					char* part3;
					memcpy(part3, 0, sizeof(argv[i + 3]) - 3);
					argin.fps = atoi(part3);
					argin.has_fps = true;
				}

				indexes.push_back(argin);
			}
				
		}
	}

	
	int camera_nums;

	RR_SHARED_PTR<Webcam_RR_impl> k = RR_MAKE_SHARED<Webcam_RR_impl>(indexes);
	/*
	// Register Local Transport
	boost::shared_ptr<RobotRaconteur::LocalTransport> t1 = boost::make_shared<RobotRaconteur::LocalTransport>();
	t1->StartServerAsNodeName("sensors.kinect2");
	RobotRaconteur::RobotRaconteurNode::s()->RegisterTransport(t1);

	// Register TCP Transport on port 8888
	boost::shared_ptr<RobotRaconteur::TcpTransport> t = boost::make_shared<RobotRaconteur::TcpTransport>();
	t->StartServer(8888);
	t->EnableNodeAnnounce(	RobotRaconteur::IPNodeDiscoveryFlags_LINK_LOCAL |
							RobotRaconteur::IPNodeDiscoveryFlags_NODE_LOCAL |
							RobotRaconteur::IPNodeDiscoveryFlags_SITE_LOCAL);
	RobotRaconteur::RobotRaconteurNode::s()->RegisterTransport(t);

	// Create the Kinect object
	

	// Register the service type with Robot Raconteur
	RobotRaconteur::RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<sensors::kinect2::sensors__kinect2Factory>());
*/
	// Register the Kinect object as a service
	
	for (int i = 0; i < k->cameras.size(); i++) {
		std::string name = "Webcam" + std::to_string(i);
		RobotRaconteur::RobotRaconteurNode::s()->RegisterService(name, "com.robotraconteur.imaging", k->cameras[i]);

	}
	
	//RobotRaconteur::RobotRaconteurNode::s()->RegisterService("KinectMultiCamera", "com.robotraconteur.imaging", k->cameras[0]);

	std::cout << "Connect to the Webcam Services at: " << std::endl;
	std::cout << "tcp://localhost:8888/opencv.webcams/Webcam{index}" << std::endl;
	std::cout << "Press enter to finish" << std::endl;
	std::getline(std::cin, dummy);
	k->ShutdownWebcams();
	return 0;
}