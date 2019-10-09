#include <boost/test/unit_test.hpp>
#include <boost/concept_check.hpp>
#include <udp/communications/SequencedComm.hpp>

#include <iostream>
#include <stdio.h>
#include <sstream>

using namespace udp;

void *sender(void *params);
void *recver(void *params);
void *sample_sender(void *params);
void *sample_recver(void *params);

volatile bool received;

struct info {
    int pid;
    std::vector<uint8_t> data;
    int bytes;
    Config conf;
};

BOOST_AUTO_TEST_CASE(send_recv)
{
    /*
    printf("Test send_recv start\n");
    std::cout << "<send_recv>" << std::endl;
    received = false;
    pthread_t threads[2];


    struct info inf_1;
    inf_1.pid = 0;

    Fragment frag;
    frag.frag_nr = 0;
    frag.id = 1;
    frag.next_frag = 0;

    FrameInfo frame_inf;
    frame_inf.ack_required = 0;
    frame_inf.c_len = 0;

    inf_1.data.push_back(frag.id & 0xff);
    inf_1.data.push_back(frag.id >> 8);
    inf_1.data.push_back(frame_inf.ack_required);
    inf_1.data.push_back(frame_inf.c_len & 0xff);
    inf_1.data.push_back(frame_inf.c_len >> 8);
    inf_1.data.push_back('t');
    inf_1.data.push_back('e');
    inf_1.data.push_back('s');
    inf_1.data.push_back('t');
    inf_1.bytes = inf_1.data.size();

    inf_1.conf.ip_addr = "127.0.0.1";
    inf_1.conf.listen_any = true;
    inf_1.conf.listen_port = 20002;
    inf_1.conf.max_fragment_size = 100;
    inf_1.conf.to_port = 20001;


    struct info inf_2;
    inf_2.pid = 1;
    inf_2.bytes = inf_1.bytes;

    inf_2.conf.ip_addr = "127.0.0.1";
    inf_2.conf.listen_any = true;
    inf_2.conf.listen_port = 20001;
    inf_2.conf.max_fragment_size = 100;
    inf_2.conf.to_port = 20002;

    if (pthread_create(&threads[1], 0, recver, &inf_2) == 0) {
        printf("thread[%d] successfully created!\n", inf_2.pid);
    } else {
        fprintf(stderr, "failed to create thread[%d]!\n", inf_2.pid);
    }

    if (pthread_create(&threads[0], 0, sender, &inf_1) == 0) {
        printf("thread[%d] successfully created!\n", inf_1.pid);
    } else {
        fprintf(stderr, "failed to create thread[%d]!\n", inf_1.pid);
    }


    for (int i = 0; i < static_cast<int>(sizeof(threads) / sizeof(pthread_t)); i++) {
        if (pthread_join(threads[i], 0) == 0) {
            printf("joined thread[%d]!\n", i);
        } else {
            fprintf(stderr, "failed to join thread[%d]!\n", i);
        }
    }

    if (inf_2.data.size() > 0) {
        std::cout << "bytes send: " << inf_1.data.size() << ", bytes received: " << inf_2.data.size() << std::endl;
        if (inf_2.data.size() > sizeof(Fragment)) {
            Fragment frag;
            memcpy(&frag, inf_2.data.data(), sizeof(Fragment));

            std::cout << "frag.id: " << frag.id << std::endl;
            std::cout << "frag.frag_nr: " << frag.frag_nr << std::endl;
            std::cout << "frag.next_frag: " << frag.next_frag << std::endl;

            inf_2.data.erase(inf_2.data.begin(), inf_2.data.begin() + sizeof(Fragment));

            if (inf_2.data.size() >= sizeof(FrameInfo)) {
                FrameInfo frame_inf;
                memcpy(&frame_inf, inf_2.data.data(), sizeof(FrameInfo));
                inf_2.data.erase(inf_2.data.begin(), inf_2.data.begin() + sizeof(FrameInfo));

                std::cout << "frame_inf.ack_required: " << frame_inf.ack_required << std::endl;
                std::cout << "frame_inf.c_len: " << frame_inf.c_len << std::endl;
                std::cout << "data: " << std::string(inf_2.data.begin(), inf_2.data.end()) << std::endl;
            }
        }
    }
    
    printf("Test send_recv end\n");
    */
}

BOOST_AUTO_TEST_CASE(register_ack_for_test)
{
    /*
    printf("Test register_ack_for_test start\n");
    
    Config conf;
    conf.init_listen = false;
    conf.init_send = false;
    conf.listen_port = 20001;
    conf.resend_after = 1000;
    conf.to_port = 20002;
    SequencedComm *comm = new SequencedComm(conf);

    comm->ack_required[0].push_back('a');
    comm->ack_required_infos[0]["127.0.0.1"].remaining_resend_tries = 1;
    comm->ack_required_infos[0]["192.168.56.101"].remaining_resend_tries = 1;

    BOOST_CHECK_EQUAL(comm->ack_required.size(), 1);
    BOOST_CHECK_EQUAL(comm->ack_required.find(0) != comm->ack_required.end(), true);

    BOOST_CHECK_EQUAL(comm->ack_required_infos.size(), 1);
    BOOST_CHECK_EQUAL(comm->ack_required_infos.find(0) != comm->ack_required_infos.end(), true);
    BOOST_CHECK_EQUAL(comm->ack_required_infos[0].size(), 2);

    comm->registerAckFor(0, "127.0.0.1");

    BOOST_CHECK_EQUAL(comm->ack_required_infos[0].size(), 1);
    BOOST_CHECK_EQUAL(comm->ack_required_infos[0].find("192.168.56.101") != comm->ack_required_infos[0].end(), true);
    BOOST_CHECK_EQUAL(comm->ack_required_infos[0].find("127.0.0.1") == comm->ack_required_infos[0].end(), true);
    
    printf("Test register_ack_for_test end\n");
    */
}

BOOST_AUTO_TEST_CASE(update_ttl_test)
{
    /*
    printf("Test update_ttl_test start\n");
    
    Config conf;
    conf.init_listen = false;
    conf.init_send = false;
    conf.listen_port = 20001;
    conf.to_port = 20002;
    SequencedComm *comm = new SequencedComm(conf);

    comm->unprocessedFrames[0].ttl = 5000;

    BOOST_CHECK_EQUAL(comm->unprocessedFrames.size(), 1);
    comm->updateTtl(5000);
    BOOST_CHECK_EQUAL(comm->unprocessedFrames.size(), 0);

    comm->unprocessedFrames[0].ttl = 10000;

    BOOST_CHECK_EQUAL(comm->unprocessedFrames.size(), 1);
    comm->updateTtl(5000);
    BOOST_CHECK_EQUAL(comm->unprocessedFrames.size(), 1);
    BOOST_CHECK_EQUAL(comm->unprocessedFrames[0].ttl, 5000);

    comm->unprocessedFrames[0].ttl = 5000;
    BOOST_CHECK_EQUAL(comm->unprocessedFrames.size(), 1);
    comm->updateTtl(10000);
    BOOST_CHECK_EQUAL(comm->unprocessedFrames.size(), 0);
    
    printf("Test update_ttl_test end\n");
    */
}

BOOST_AUTO_TEST_CASE(update_unconfirmed_frames_test)
{
    /*
    printf("Test update_unconfirmed_frames_test start\n");
    
    Config conf;
    conf.init_listen = false;
    conf.init_send = false;
    conf.repeat_packets = false;
    conf.resend_after = 1;
    conf.resend_after = 1000;
    conf.listen_port = 20001;
    conf.to_port = 20002;
    SequencedComm *comm = new SequencedComm(conf);

    // erase incomplete frames when update results to remaining_resend_tries == 0
    comm->ack_required_infos[0]["127.0.0.1"].until_resent = 5000;
    comm->ack_required_infos[0]["127.0.0.1"].remaining_resend_tries = 0;

    BOOST_CHECK_EQUAL(comm->ack_required_infos.size(), 1);
    comm->updateUnconfirmedFrames(5000);
    BOOST_CHECK_EQUAL(comm->ack_required_infos.size(), 0);

    // increase remaining_resend_tries
    comm->ack_required_infos[0]["127.0.0.1"].until_resent = 5000;
    comm->ack_required_infos[0]["127.0.0.1"].remaining_resend_tries = 2;

    BOOST_CHECK_EQUAL(comm->ack_required_infos.size(), 1);
    comm->updateUnconfirmedFrames(5000);
    BOOST_CHECK_EQUAL(comm->ack_required_infos.size(), 1);
    BOOST_CHECK_EQUAL(comm->ack_required_infos[0]["127.0.0.1"].remaining_resend_tries, 1);

    // don not erase entry when there is another client left with enough
    // remaining_resend_tries for frame
    std::cout << std::endl << std::endl;
    comm->ack_required_infos[0]["127.0.0.1"].until_resent = 5000;
    comm->ack_required_infos[0]["127.0.0.1"].remaining_resend_tries = 1;
    comm->ack_required_infos[0]["192.168.56.101"].until_resent = 5000;
    comm->ack_required_infos[0]["192.168.56.101"].remaining_resend_tries = 0;

    comm->ack_required[0].push_back('a');
    BOOST_CHECK_EQUAL(comm->ack_required.size(), 1);
    BOOST_CHECK_EQUAL(comm->ack_required.find(0) != comm->ack_required.end(), true);

    BOOST_CHECK_EQUAL(comm->ack_required_infos.size(), 1);
    BOOST_CHECK_EQUAL(comm->ack_required_infos[0].size(), 2);
    comm->updateUnconfirmedFrames(5000);
    BOOST_CHECK_EQUAL(comm->ack_required_infos.size(), 1);
    BOOST_CHECK_EQUAL(comm->ack_required_infos[0]["127.0.0.1"].remaining_resend_tries, 0);
    BOOST_CHECK_EQUAL(comm->ack_required_infos[0]["127.0.0.1"].until_resent, conf.resend_after);
    BOOST_CHECK_EQUAL(comm->ack_required_infos[0].size(), 1);
    BOOST_CHECK_EQUAL(comm->mComInfo.numFramesRepeated, 1);
    
    printf("Test update_unconfirmed_frames_test end\n");
    */
}

BOOST_AUTO_TEST_CASE(send_recv_sample_test)
{
    /*
    printf("Test send_recv_sample_test start\n");
    
    received = false;
    pthread_t threads[2];

    
    struct info inf_1;
    inf_1.pid = 0;


    inf_1.data.push_back('t');
    inf_1.data.push_back('e');
    inf_1.data.push_back('s');
    inf_1.data.push_back('t');
    inf_1.bytes = inf_1.data.size();

    inf_1.conf.ip_addr = "127.0.0.1";
    inf_1.conf.max_fragment_size = 100;
    inf_1.conf.listen_port = 20002;
    inf_1.conf.to_port = 20012;
    inf_1.conf.init_listen = false;


    struct info inf_2;
    inf_2.pid = 1;
    inf_2.conf.ip_addr = "127.0.0.1";
    inf_2.conf.listen_port = 20012;
    inf_2.conf.to_port = 20002;
    inf_2.conf.init_send = false;
    inf_2.conf.send_acks = false;
    inf_2.conf.listen_any = true;

    if (pthread_create(&threads[1], 0, sample_recver, &inf_2) == 0) {
        printf("thread[%d] successfully created!\n", inf_2.pid);
    } else {
        fprintf(stderr, "failed to create thread[%d]!\n", inf_2.pid);
    }

    if (pthread_create(&threads[0], 0, sample_sender, &inf_1) == 0) {
        printf("thread[%d] successfully created!\n", inf_1.pid);
    } else {
        fprintf(stderr, "failed to create thread[%d]!\n", inf_1.pid);
    }


    for (int i = 0; i < static_cast<int>(sizeof(threads) / sizeof(pthread_t)); i++) {
        if (pthread_join(threads[i], 0) == 0) {
            printf("joined thread[%d]!\n", i);
        } else {
            fprintf(stderr, "failed to join thread[%d]!\n", i);
        }
    }
    
    printf("Test send_recv_sample_test end\n");
    
    */
}

void *sender(void *params)
{
    /*
    SequencedComm *comm = new SequencedComm(((struct info *)params)->conf);

    while (!received) {
        std::stringstream ss;
        for (std::vector<uint8_t>::const_iterator it = ((struct info *)params)->data.begin(); it != ((struct info *)params)->data.end(); it++) {
            ss << *it;
        }

        std::cout << "sending [" << ss.str().size() << "/" << ((struct info *)params)->data.size() << "/" << ((struct info *)params)->bytes << "]" << ": " << ss.str() << std::endl;
        ss.clear();
        
        // int bytes_sent = 
        comm->sendData(((struct info *)params)->data, ((struct info *)params)->bytes, "127.0.0.1", false);

        for (std::vector<uint8_t>::const_iterator it = ((struct info *)params)->data.begin(); it != ((struct info *)params)->data.end(); it++) {
            ss << *it;
        }

        if (!received) sleep(1);
    }

    delete(comm);
    */
    return NULL;
    
}

void *sample_sender(void *params)
{
    /*
    SequencedComm *comm = new SequencedComm(((struct info *)params)->conf);
    std::cout << "sender to_port: " << ((struct info *)params)->conf.to_port << std::endl;
    std::cout << "sender ip_addr: " << ((struct info *)params)->conf.ip_addr << std::endl;
    std::cout << "sender listen_port: " << ((struct info *)params)->conf.listen_port << std::endl;
    sleep(1);

    int counter = 0;
    while (!received && counter < 10) {
        std::vector<uint8_t> sendBuffer = { 't', 'e', 's', 't' };

        std::map<CONTROL_IDS, std::string> ctrl;
        ctrl.insert(std::make_pair(PORT_NAME, "sample_port"));
        
        bool needs_ack = false;
        SendInfo s_inf;
        
        std::vector<std::string> toAddrs;
        
        s_inf = comm->sendDataSample(ctrl, sendBuffer, needs_ack, toAddrs);

        std::string send_dat_str = std::string(sendBuffer.begin(), sendBuffer.end());
        std::cout << "sample_sender: " << send_dat_str << "[" << s_inf.bytes_sent << " bytes]" << std::endl;
        if (!received) sleep(1);
        
        counter++;
    }

    delete(comm);
    
    */
    return NULL;
}

void *recver(void *params)
{
    /*
    SequencedComm *comm = new SequencedComm(((struct info *)params)->conf);
    int bytes_rcvd = 0;
    std::string src_addr;

    char buffer[((struct info *)params)->conf.max_fragment_size];

    //while ((bytes_rcvd = comm->receive_data(src_addr, &buffer[0], ((struct info *)params)->conf.max_fragment_size)) <= 0);
    ((struct info *)params)->data.assign(buffer, buffer + bytes_rcvd);

    std::stringstream ss;
    for (std::vector<uint8_t>::const_iterator it = ((struct info *)params)->data.begin(); it != ((struct info *)params)->data.end(); it++) {
        ss << *it;
    }

    std::cout << "received[" << bytes_rcvd << " bytes] " << ss.str() << std::endl;
    received = true;

    delete(comm);
    
    */
    return NULL;
}

void *sample_recver(void *params)
{
    /*
    SequencedComm *comm = new SequencedComm(((struct info *)params)->conf);
    std::cout << "recv to_port: " << ((struct info *)params)->conf.to_port << std::endl;
    std::cout << "recv ip_addr: " << ((struct info *)params)->conf.ip_addr << std::endl;
    std::cout << "recv listen_port: " << ((struct info *)params)->conf.listen_port << std::endl;
    sleep(1);

    SampleData sampleData;

    // receiveSample returns false when receiving ack
    int counter = 0;
    while (!comm->receiveSample(sampleData) && counter++ < 10) {
        std::string data_to_string = std::string(sampleData.data.begin(), sampleData.data.end());
        std::cout << "sample_recver: " << sampleData.port_name << " -> " << data_to_string << std::endl;
        sleep(1);
    }


    received = true;

    delete(comm);
    
    */
    return NULL;
}
