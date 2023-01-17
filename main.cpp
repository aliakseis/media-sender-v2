/*
 * libdatachannel media sender example
 * Copyright (c) 2020 Staz Modrzynski
 * Copyright (c) 2020 Paul-Louis Ageneau
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; If not, see <http://www.gnu.org/licenses/>.
 */

#include "rtc/rtc.hpp"
#include "rtc/rtp.hpp"

#include "cameraman.h"


#include <cstddef>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>



#include <nlohmann/json.hpp>


#include <gst/gst.h>

#include <gst/app/gstappsink.h>   

 /* bar-menu.c
 Copyright (c) 2011, Frank Cox <theatre@melvilletheatre.com>
 December 7, 2011
 All rights reserved.
 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:
	 * Redistributions of source code must retain the above copyright
	   notice, this list of conditions and the following disclaimer.
	 * Redistributions in binary form must reproduce the above copyright
	   notice, this list of conditions and the following disclaimer in the
	   documentation and/or other materials provided with the distribution.
 THIS SOFTWARE IS PROVIDED BY FRANK COX ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL FRANK COX BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 OF SUCH DAMAGE.
  */

#include <curses.h>

static int barmenu(const std::vector<std::string>& array, const int row, const int col, const int width, int menulength, int selection = 0)
{
	int offset = 0;
	int ky = 0;
	char formatstring[7];
	curs_set(0);

	const int arraylength = array.size();

	if (arraylength < menulength) {
		menulength = arraylength;
	}

	if (selection > menulength) {
		offset = selection - menulength + 1;
	}

	snprintf(formatstring, sizeof(formatstring) / sizeof(formatstring[0]), "%%-%ds", width); // remove - sign to right-justify the menu items

	while (ky != 27)
	{
		for (int counter = 0; counter < menulength; counter++)
		{
			if (counter + offset == selection) {
				attron(A_REVERSE);
			}
			mvprintw(row + counter, col, formatstring, array[counter + offset].c_str());
			attroff(A_REVERSE);
		}

		ky = getch();

		switch (ky)
		{
		case KEY_UP:
			if (selection)
			{
				selection--;
				if (selection < offset) {
					offset--;
				}
			}
			break;
		case KEY_DOWN:
			if (selection < arraylength - 1)
			{
				selection++;
				if (selection > offset + menulength - 1) {
					offset++;
				}
			}
			break;
		case KEY_HOME:
			selection = 0;
			offset = 0;
			break;
		case KEY_END:
			selection = arraylength - 1;
			offset = arraylength - menulength;
			break;
		case KEY_PPAGE:
			selection -= menulength;
			if (selection < 0) {
				selection = 0;
			}
			offset -= menulength;
			if (offset < 0) {
				offset = 0;
			}
			break;
		case KEY_NPAGE:
			selection += menulength;
			if (selection > arraylength - 1) {
				selection = arraylength - 1;
			}
			offset += menulength;
			if (offset > arraylength - menulength) {
				offset = arraylength - menulength;
			}
			break;
		case 10: //enter
			return selection;
			break;
		case KEY_F(1): // function key 1
			return -1;
		case 27: //esc
				// esc twice to get out, otherwise eat the chars that don't work
				//from home or end on the keypad
			ky = getch();
			if (ky == 27)
			{
				curs_set(0);
				mvaddstr(9, 77, "   ");
				return -1;
			}

			if (ky == '[')
			{
				getch();
				getch();
			}
			else
				ungetch(ky);
		}
	}
	return -1;
}











//#ifdef _WIN32
//#define _WINSOCK_DEPRECATED_NO_WARNINGS
//#include <winsock2.h>
//#else
//#include <arpa/inet.h>
//#include <netinet/in.h>
//#include <sys/socket.h>
//typedef int SOCKET;
//#endif




using nlohmann::json;

//const int BUFFER_SIZE = 2048;

int main(int argc, char *argv[]) {
	try {


		gst_init(&argc, &argv);

		const auto cameraDescriptions = getCameraDescriptions();
		if (cameraDescriptions.empty())
		{
			std::cerr << "No cameras" << std::endl;
			return -1;
		}


		initscr();
		noecho();
		keypad(stdscr, TRUE);

		std::vector<std::string> items;
		items.reserve(cameraDescriptions.size());
		for (auto& desc : cameraDescriptions)
		{
			items.push_back(desc.description);
		}

		const int row = 1;
		const int col = 2;
		const int menuwidth = 70;
		const int menulength = 20;

		mvprintw(0, 0, "Please select a camera:");

		int selection = barmenu(items, row, col, menuwidth, menulength);
		if (selection == -1)
		{
			return 0;
		}

		const auto& desc = cameraDescriptions[selection];

		items.clear();
		for (auto& mode : desc.modes)
		{
			items.push_back(mode.getDescr());
		}

		refresh();

		mvprintw(0, 0, "Please select a camera mode:");

		selection = barmenu(items, row, col, menuwidth, menulength);
		if (selection == -1)
		{
			return 0;
		}

		const auto& mode = desc.modes[selection];

		endwin();


		//rtc::InitLogger(plog::Severity::debug); //rtc::LogLevel::Debug);
		rtc::InitLogger(rtc::LogLevel::Debug);
		auto pc = std::make_shared<rtc::PeerConnection>();

		pc->onStateChange(
		    [](rtc::PeerConnection::State state) { std::cout << "State: " << state << std::endl; });

		pc->onGatheringStateChange([pc](rtc::PeerConnection::GatheringState state) {
			std::cout << "Gathering State: " << state << std::endl;
			if (state == rtc::PeerConnection::GatheringState::Complete) {
				auto description = pc->localDescription();
				json message = {{"type", description->typeString()},
				                {"sdp", std::string(description.value())}};
				std::cout << message << std::endl;
			}
		});

		//SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);
		//struct sockaddr_in addr = {};
		//addr.sin_family = AF_INET;
		//addr.sin_addr.s_addr = inet_addr("127.0.0.1");
		//addr.sin_port = htons(6000);

		//if (bind(sock, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) < 0)
		//	throw std::runtime_error("Failed to bind UDP socket on 127.0.0.1:6000");

		//int rcvBufSize = 212992;
		//setsockopt(sock, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char *>(&rcvBufSize),
		//           sizeof(rcvBufSize));

		const rtc::SSRC ssrc = 42;
		rtc::Description::Video media("video", rtc::Description::Direction::SendOnly);
		media.addH264Codec(96); // Must match the payload type of the external h264 RTP stream
		media.addSSRC(ssrc, "video-send");
		auto track = pc->addTrack(media);

		pc->setLocalDescription();

		std::cout << "RTP video stream expected on localhost:6000" << std::endl;
		std::cout << "Please copy/paste the answer provided by the browser: " << std::endl;
		std::string sdp;
		std::getline(std::cin, sdp);

		json j = json::parse(sdp);
		rtc::Description answer(j["sdp"].get<std::string>(), j["type"].get<std::string>());
		pc->setRemoteDescription(answer);

		// Receive from UDP
		//char buffer[BUFFER_SIZE];
		//int len;
		//while ((len = recv(sock, buffer, BUFFER_SIZE, 0)) >= 0) {
		//	if (len < sizeof(rtc::RtpHeader) || !track->isOpen())
		//		continue;

		//	auto rtp = reinterpret_cast<rtc::RtpHeader *>(buffer);
		//	rtp->setSsrc(ssrc);

		//	track->send(reinterpret_cast<const std::byte *>(buffer), len);
		//}


		// gst-launch-1.0 ksvideosrc ! video/x-raw,width=640,height=480 ! videoconvert ! queue ! x264enc tune=zerolatency bitrate=1000 key-int-max=30 ! video/x-h264, profile=constrained-baseline ! rtph264pay pt=96 mtu=1200 ! udpsink host=127.0.0.1 port=6000

		std::ostringstream s;

		s << desc.launchLine
			<< " ! video/x-raw,format=" << mode.format << ",width=" << mode.w << ",height=" << mode.h << ",framerate=" << mode.den << "/" << mode.num
			//<< " ! videoconvert ! appsink";
			<< " ! videoconvert ! queue ! x264enc tune=zerolatency bitrate=1000 key-int-max=30 ! video/x-h264, profile=constrained-baseline ! rtph264pay pt=96 mtu=1200 ! appsink name=sink";

		GError *error = NULL;
		auto pipeline = gst_parse_launch(s.str().c_str(), &error);


		if (error != NULL) {
			g_print("could not construct pipeline: %s\n", error->message);
			g_error_free(error);
			exit(-1);
		}

		/* get sink */
		auto sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");

		/* set to PAUSED to make the first frame arrive in the sink */
		auto ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);// GST_STATE_PAUSED);
		switch (ret) {
		case GST_STATE_CHANGE_FAILURE:
			g_print("failed to play the file\n");
			exit(-1);
		case GST_STATE_CHANGE_NO_PREROLL:
			/* for live sources, we need to set the pipeline to PLAYING before we can
			* receive a buffer. We don’t do that yet */
			g_print("live sources not supported yet\n");
			exit(-1);
		default:
			break;
		}

		/* This can block for up to 5 seconds. If your machine is really overloaded,
		* it might time out before the pipeline prerolled and we generate an error. A
		* better way is to run a mainloop and catch errors there. */
		ret = gst_element_get_state(pipeline, NULL, NULL, 5 * GST_SECOND);
		if (ret == GST_STATE_CHANGE_FAILURE) {
			g_print("failed to play the file\n");
			exit(-1);
		}


		for (;;)
		{ 
			// bail out if EOS
			if (gst_app_sink_is_eos(GST_APP_SINK(sink)))
				return false;


			auto sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));

			if (!sample)
				return false;

			auto buffer = gst_sample_get_buffer(sample);


			GstMapInfo map;
			if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
				// Copy the data 
				//data = new QByteArray((char *)map.data, map.size);
				//vs->emitVideo(data);

				auto len = map.size;
				if (len < sizeof(rtc::RtpHeader) || !track->isOpen())
					continue;

				auto rtp = reinterpret_cast<rtc::RtpHeader *>(map.data);
				rtp->setSsrc(ssrc);

				track->send(reinterpret_cast<const std::byte *>(map.data), len);


				gst_buffer_unmap(buffer, &map);
			}
			else {
				std::cerr << "Error with gst_buffer_map\n";
			}

			gst_sample_unref(sample);
		}



	} catch (const std::exception &e) {
		std::cerr << "Error: " << e.what() << std::endl;
	}
}
