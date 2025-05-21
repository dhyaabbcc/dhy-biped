#ifndef LOGDATA_H
#define LOGDATA_H

#include <fstream>
#include <iostream>
#include <vector>
#include <array>
#include <unordered_map>
#include <arpa/inet.h>

class DataLogger
{

public:
    DataLogger(const DataLogger &) = delete;
    static DataLogger &GET()
    {
        static DataLogger S_logger;
        return S_logger;
    }

    void Savedata()
    {

        // for (auto pair : Index)
        // {
        //     if (pair.first.substr(0, 5) == std::string("joint"))
        //     {
        //         auto ARRAY = (std::array<float, 12> *)pair.second;
        //         if (openFile(pair.first))
        //         {
        //             for (float member : *ARRAY)
        //                 file << member << '  ';
        //             file << '\n';
        //         }
        //         file.close();
        //     }
        //     else
        //     {
        //         auto value = (float *)pair.second;
        //         if (openFile(pair.first))
        //             file << *value << '\n';
        //         file.close();
        //     }
        // }

        // if (openFile("legstate.txt"))
        // {

        //     file << R_legstate << "  " << L_legstate;
        //     file << '\n';
        //     file.close();
        // }
    }
    void Senddata()
    {

        if(!(counter%10))
        {
        std::string sendbuff="d:";
        // for(int i=0;i<12;i++)
        // {
        //     sendbuff+=std::to_string(i);
        //     if (i < 11) sendbuff += ",";
        //     else sendbuff += "\n";
        // }
        sendbuff+=std::to_string(vxBody_des);
        sendbuff+=",";
        sendbuff+=std::to_string(vxBody);
        sendbuff+=",";
        sendbuff+=std::to_string(vyBody_des);
        sendbuff+=",";
        sendbuff+=std::to_string(vyBody);
        sendbuff+=",";
        sendbuff+=std::to_string(vzBody_des);
        sendbuff+=",";
        sendbuff+=std::to_string(vzBody);
        sendbuff+=",";

        sendbuff+=std::to_string(pxBody);
        sendbuff+=",";
        sendbuff+=std::to_string(pyBody);
        sendbuff+=",";
        sendbuff+=std::to_string(pzBody);
        sendbuff+=",";

        sendbuff+=std::to_string(rBody);
        sendbuff+=",";
        sendbuff+=std::to_string(pBody);
        sendbuff+=",";
        sendbuff+=std::to_string(yBody);
       // sendbuff+=",";

        for(int i=0;i<6;i++)
        {
             sendbuff+=",";
            sendbuff+=std::to_string(Fmpc[i]);
           
        }

        for(int i=0;i<6;i++)
        {
            sendbuff+=",";
            sendbuff+=std::to_string(Fwbc[i]);
            
        }
        for(int i=0;i<6;i++)
        {
            sendbuff+=",";
            sendbuff+=std::to_string(10*jointTorque[i]);
        }

        sendbuff+='\n';

        const char * str=sendbuff.c_str();
        char sendBuf[512];
        std::cout<<"SEND data!\n"<<'\n';
        //sprintf(sendBuf, "d:%f,%f,%f,%f\n",jointpos[3],jointposdes[3],jointpos[2],jointposdes[2]);
       // sprintf(sendBuf, "d:%f,%f\n",0,0);
        //sendto(fd, sendBuf, strlen(sendBuf) + 1, 0, (struct sockaddr *)&saddr, sizeof(saddr));
        sendto(fd, str, strlen(str) + 1, 0, (struct sockaddr *)&saddr, sizeof(saddr));
        }
        counter++;
    }
    ~DataLogger()
    {
        if (file.is_open())
        {
            file.close();
        }
    }

    float vxBody, vxBody_des, vyBody, vyBody_des, vzBody, vzBody_des, vx_ly;
    float pxBody, pyBody, pzBody, pxBody_des, pyBody_des, pzBody_des;
    float rBody, pBody, yBody, rBody_des, pBody_des, yBody_des;
    float Fwbc[6];
    float Fmpc[6];
    
   std::array<float, 12> jointpos;
   std::array<float, 12> jointvel;
    std::array<float, 12> jointposdes;
   std::array<float, 12> jointveldes;
    std::array<float, 12> jointTorque;
    bool R_legstate, L_legstate; // 腿处于摆动或支撑状态

private:
    std::ofstream file;
    std::unordered_map<std::string, void *> Index;
    std::vector<std::string> filepath;
    std::string wholefile;
    const std::string prefix = "datalog/";
    int counter=0;
    int sock;
    struct sockaddr_in server;
    char message[1000];
    int fd = -1;
    struct sockaddr_in saddr;
    DataLogger()
    {
        fd = socket(PF_INET, SOCK_DGRAM, 0);

        if (fd == -1)
        {
            perror("socket");
            exit(-1);
        }

        // 服务器的地址信息

        saddr.sin_family = AF_INET;
        saddr.sin_port = htons(9999);
        inet_pton(AF_INET, "192.168.219.1", &saddr.sin_addr.s_addr);

        for(int i=0;i<6;i++)
         Fwbc[i]=0;

        // filepath.push_back("bodyvelxdes.txt");
        // Index[filepath.back()] = (void *)&vxBody;
        // filepath.push_back("bodyvelx.txt");
        // Index[filepath.back()] = (void *)&vxBody_des;
        // filepath.push_back("usrvxly.txt");
        // Index[filepath.back()] = (void *)&vx_ly;
        // filepath.push_back("jointpos.txt");
        // Index[filepath.back()] = (void *)&jointpos;
        // filepath.push_back("jointTorque.txt");
        // Index[filepath.back()] = (void *)&jointTorque;
        // filepath.push_back("jointvel.txt");
        // Index[filepath.back()] = (void *)&jointvel;
        // filepath.push_back("bodyvelydes.txt");
        // Index[filepath.back()] = (void *)&vyBody_des;
        // filepath.push_back("bodyvely.txt");
        // Index[filepath.back()] = (void *)&vyBody;
        // filepath.push_back("bodyvelz.txt");
        // Index[filepath.back()] = (void *)&vzBody;
        // filepath.push_back("bodyvelzdes.txt");
        // Index[filepath.back()] = (void *)&vzBody_des;
        // filepath.push_back("legstate.txt");
        // filepath.push_back("jointposdes.txt");
        // Index[filepath.back()] = (void *)&jointposdes;
        // filepath.push_back("jointveldes.txt");
        // Index[filepath.back()] = (void *)&jointveldes;
        // filepath.push_back("bodypx.txt");
        // Index[filepath.back()] = (void *)&pxBody;
        // filepath.push_back("bodypy.txt");
        // Index[filepath.back()] = (void *)&pyBody;
        // filepath.push_back("bodypz.txt");
        // Index[filepath.back()] = (void *)&pzBody;
        // filepath.push_back("bodyr.txt");
        // Index[filepath.back()] = (void *)&rBody;
        // filepath.push_back("bodyp.txt");
        // Index[filepath.back()] = (void *)&pBody;
        // filepath.push_back("bodyy.txt");
        // Index[filepath.back()] = (void *)&yBody;

        // for (auto rm_file : filepath)
        // {
        //     std::ofstream file1;
        //     wholefile = prefix + rm_file;
        //     file1.open(wholefile);
        //     // if (file1.is_open())
        //     // {
        //     //     std::cout<<"create file"<<wholefile<<std::endl;
        //     // //    file1<<"createfile"<<'\n';
        //     // }
        //     // else
        //     // {
        //     //     std::cout << "Failed to create file: " << wholefile << std::endl;
        //     // }
        // }
    }
    bool openFile(const std::string &filename)
    {

        file.open(prefix + filename, std::ios::app);
        if (!file.is_open())
        {
            std::cout << "无法打开文件！" << prefix + filename << std::endl;
            return false;
        }
        return true;
    }
};

#endif