// -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*-

#include <ns3/log.h>
#include <ns3/core-module.h>
#include <ns3/lr-wpan-mac.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/lr-wpan-csmaca.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/packet.h>
#include <ns3/basic-energy-source.h>
#include <ns3/simple-device-energy-model.h>
#include <ns3/basic-energy-harvester.h>
#include <ns3/mobility-module.h>
#include <ns3/position-allocator.h>

#include <vector>
#include <random>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <sstream>

/**********************************************************/

using namespace ns3;

class Energy {

private:
	double energy;
	double packetsSentCount = 0;

public:
	Energy(double energy)
	{
		this->energy = energy;
	}

	double GetCurrentEnergy()
	{
		// Текущее значение энергии узла
		return energy;
	}

	void HarvestEnergy(int bytes)
	{
		// Вычитаем энергию
		energy -= (0.005 * bytes);
	}

	double GetPacketsSentCount()
	{
		// Кол-во отправленных пакетов
		return packetsSentCount;
	}

	void IncreasePacketsSentCount()
	{
		// Увеличиваем число отправленных пакетов
		packetsSentCount++;
	}
};

class BoxField {

private:
	int x;
	int y;
	int l;
	int nodesCount;
	int useCompress;
	NodeContainer nodeContainer;
	std::vector<std::shared_ptr<Energy>> energyContainer;

	void createBox() {
		// Создаем сервер
		Ptr<Node> serverNode = CreateObject<Node>();
		// Создаем девайс сервера
		Ptr<LrWpanNetDevice> serverDevice = CreateObject<LrWpanNetDevice>();
		// Задаем девайсу MAC = 00:00
		serverDevice->SetAddress(Mac16Address("00:00"));

		// Объект csma
		LrWpanMacStateCallback stateCallback;
		stateCallback = MakeBoundCallback(&BoxField::LrWpanMacState, this);
		serverDevice->GetCsmaCa()->SetMacMinBE(0);
		serverDevice->GetCsmaCa()->SetLrWpanMacStateCallback(stateCallback);


		// Объект канала связи
		Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel>();
		Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel>();
		Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
		channel->AddPropagationLossModel(propModel);
		channel->SetPropagationDelayModel(delayModel);

		MobilityHelper serverMobility;
		// Задаем модель расположения сервера как ConstantPositionMobilityModel
		serverMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
		// Объект аллокатора
		Ptr<ListPositionAllocator> serverNodesPositionAlloc = CreateObject<ListPositionAllocator>();
		// Устанавливаем позицию для сервера (x,y,z)
		serverNodesPositionAlloc->Add(Vector(0, y / 2, 0));
		serverMobility.SetPositionAllocator(serverNodesPositionAlloc);
		// Устанавливаем модель расположения для сервера
		serverMobility.Install(serverNode);

		// Задаем общий канал связи для всех узлов и сервера
		serverDevice->SetChannel(channel);
		serverNode->AddDevice(serverDevice);

		// Задаем растояние от сервера = l
		int startOffsetX = l;
		// Задаем максимальный x
		int endOffsetX = l + x;

		// Генерация рандома для x
		std::random_device rdX;
		std::mt19937 genX(rdX());
		std::uniform_int_distribution<> disX(startOffsetX, endOffsetX);

		// Генерация рандома для y
		std::random_device rdY;
		std::mt19937 genY(rdY());
		std::uniform_int_distribution<> disY(0, y);

		// Создаем узлы
		for (int i = 1; i <= nodesCount; i++) {
			// Рандомный X
			int positionX = disX(genX);
			// Рандомный Y
			int positionY = disY(genY);

			char mac[5];
			sprintf(mac, "00:%02d", i);

			// Создаем узлы
			Ptr<Node> node = CreateObject<Node>();
			// Создаем девайс
			Ptr<LrWpanNetDevice> device = CreateObject<LrWpanNetDevice>();

			// Задаем девайсу MAC = 00:%02d, где %02d = номер узла с ведущим нулем
			device->SetAddress(Mac16Address(mac));
			// Задаем общий канал связи для всех узлов
			device->SetChannel(channel);

			device->GetCsmaCa()->SetMacMinBE(0);
			device->GetCsmaCa()->SetLrWpanMacStateCallback(stateCallback);

			MobilityHelper mobility;
			// Задаем модель расположения узлов как ConstantPositionMobilityModel
			mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
			// Объект аллокатора
			Ptr<ListPositionAllocator> nodesPositionAlloc = CreateObject<ListPositionAllocator>();
			// Устанавливаем позицию для узла (x,y,z)
			nodesPositionAlloc->Add(Vector(positionX, positionY, 0));
			mobility.SetPositionAllocator(nodesPositionAlloc);
			// Устанавливаем модель расположения для узла
			mobility.Install(node);

			// Добавляем девайс к узлу
			node->AddDevice(device);

			McpsDataConfirmCallback dataConfirmCallback;
			dataConfirmCallback = MakeBoundCallback(&BoxField::DataConfirm, this, node);
			device->GetMac()->SetMcpsDataConfirmCallback(dataConfirmCallback);

			// Создаем эклемпляр класса Energy для узла
			// Начальное значение 2 джоуля
			energyContainer.push_back(std::shared_ptr<Energy>(new Energy(2)));

			NS_LOG_UNCOND("Узел #" << i << " добавлена, координаты (" << (positionX - l) << "," << positionY << ")"
				<< " расстояние до сервера: " << device->GetPhy()->GetMobility()->GetDistanceFrom(serverDevice->GetPhy()->GetMobility()));

			// Добавляем ноду в контейнер нод
			nodeContainer.Add(node);
		}
	}

public:
	static void DataIndication(BoxField *boxField, Ptr<Node> node, McpsDataIndicationParams params, Ptr<Packet> p);
	static void DataConfirm(BoxField *boxField, Ptr<Node> node, McpsDataConfirmParams params);
	static void LrWpanMacState(BoxField *boxField, ns3::LrWpanMacState state);

	BoxField(int x, int y, int l, int nodesCount, int useCompress)
	{
		this->x = x;
		this->y = y;
		this->l = l;
		this->nodesCount = nodesCount;
		this->useCompress = useCompress;

		if (useCompress > 0) {
			NS_LOG_UNCOND("Компрессия включена.");
		} else {
			NS_LOG_UNCOND("Компрессия выключена.");
		}

		NS_LOG_UNCOND("Количество узлов: " << nodesCount << ", " << "Расстояние до сервера: " << l);

		createBox();
	}

	static void SendOnePacket(Ptr<Node> node, std::vector<std::shared_ptr<Energy>> energyContainer, int l, int useCompress)
	{
		// Получаем объект девайс из объекта узел
		Ptr<LrWpanNetDevice> device = DynamicCast<LrWpanNetDevice>(node->GetDevice(0));

		// Задаем параметры для отправки
		// https://www.nsnam.org/doxygen/structns3_1_1_mcps_data_request_params.html
		McpsDataRequestParams params;
		// Куда отправляем - MAC сервера 00:00
		params.m_dstAddr = Mac16Address("00:00");
		params.m_srcAddrMode = SHORT_ADDR;
		params.m_dstAddrMode = SHORT_ADDR;
		params.m_dstPanId = 0;
		params.m_msduHandle = 0;
		params.m_txOptions = TX_OPTION_ACK;

		//long packetBytes = createMessage(node, useCompress);
		long packetBytes = 50;

		//NS_LOG_UNCOND("Отправлен пакет размером " << packetBytes << " байт" << " с узла #" << node->GetId());

		// Создаем пакет
		Ptr<Packet> packet = Create<Packet>(packetBytes);

		// Отправляем
		device->GetMac()->McpsDataRequest(params, packet);


		// Устанавливаем планировщик на отправку пакета через 5 секунд
		Simulator::ScheduleWithContext(node->GetId(), Seconds(5), &BoxField::SendOnePacket, node, energyContainer, l, useCompress);
	}

	void SendPackets()
	{
		// Инициируем начальную отправку для всех узлов
		for (unsigned i = 0; i < nodeContainer.GetN(); i++) {
			// Получаем узел из контейнера
			Ptr<Node> node = nodeContainer.Get(i);
			// Устанавливаем планировщик на отправку пакета прямо сейчас
			Simulator::ScheduleWithContext(node->GetId(), Seconds(0), &BoxField::SendOnePacket, node, energyContainer, l, useCompress);
		}
	}

	void OnPacketReceive(Ptr<Node> node)
	{
		int index = node->GetId() - 1;
		long packetBytes = 50; 

		// Получаем объект девайс из объекта узел
		Ptr<LrWpanNetDevice> device = DynamicCast<LrWpanNetDevice>(node->GetDevice(0));
		// Получаем объект энергии из объекта контейнера по индексу узла
		Energy *energy = energyContainer.at(index).get();

		// Увеличиваем число отправленных пакетов для текущего узла
		energy->IncreasePacketsSentCount();
		// Вычитаем энергию на отправку
		energy->HarvestEnergy(packetBytes);

		if (energy->GetCurrentEnergy() <= 0) {
			Simulator::Stop();
			NS_LOG_UNCOND(" Успешно переданных сообщений: " << energy->GetPacketsSentCount() << std::endl << " Вышел из строя узел #" << node->GetId() << ", координаты ("
				<< device->GetPhy()->GetMobility()->GetPosition().x - l << ", "
				<< device->GetPhy()->GetMobility()->GetPosition().y << ")");
		}
	}
};

void BoxField::DataIndication(BoxField *boxField, Ptr<Node> node, McpsDataIndicationParams params, Ptr<Packet> p)
{
	NS_LOG_UNCOND("DataIndication " << node->GetId());
}

void BoxField::DataConfirm(BoxField *boxField, Ptr<Node> node, McpsDataConfirmParams params)
{

	//NS_LOG_UNCOND("Данные с узла #" << node->GetId() << " приняты сервером");

	boxField->OnPacketReceive(node);
}

void BoxField::LrWpanMacState(BoxField *boxField, ns3::LrWpanMacState state)
{
	NS_LOG_UNCOND("LrWpanMacState " << state);
}


int main(int argc, char *argv[])
{
	int x = 50;
	int y = 50;
	int l = 10;
	int nodesCount = 1;
	int useCompress = 1;

	// Агрументы командной строки
	CommandLine cmd;
	cmd.AddValue("x", "x", x);
	cmd.AddValue("y", "y", y);
	cmd.AddValue("l", "L", l);
	cmd.AddValue("n", "Nodes count", nodesCount);
	cmd.AddValue("c", "Use compress", useCompress);
	cmd.Parse(argc, argv);

	// Создаем объект BoxField
	BoxField box = BoxField(x, y, l, nodesCount, useCompress);
	// Отправляем первые пакеты для всех узлов
	box.SendPackets();

	// Запускаем симулятор
	Simulator::Run();
	// Убиваем симулятор после того как выполнились все запланированные задачи
	Simulator::Destroy();

	return 0;
}
