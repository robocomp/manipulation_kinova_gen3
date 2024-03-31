import time
import Ice
import IceStorm
from rich.console import Console, Text
console = Console()


Ice.loadSlice("-I ./src/ --all ./src/JoystickAdapter.ice")
import RoboCompJoystickAdapter
Ice.loadSlice("-I ./src/ --all ./src/KinovaArm.ice")
import RoboCompKinovaArm

class AxisList(list):
    def __init__(self, iterable=list()):
        super(AxisList, self).__init__(iterable)

    def append(self, item):
        """
        appends an item instance to a list of AxisParam objects for RoboCompJoystickAdapters,
        checks that the input is an AxisParam instance, and calls the superclass
        method `append`.

        Args:
            item (`RoboCompJoystickAdapter.AxisParams`.): axis parameter to be
                appended to the list of axes maintained by the `AxisList` instance.
                
                		- Isinstance() check passes since `item` is of type `RoboCompJoystickAdapter.AxisParams`.
                		- `super().append(item)` adds the item to the list of Axis objects
                maintained by the `AxisList` instance.
                

        """
        assert isinstance(item, RoboCompJoystickAdapter.AxisParams)
        super(AxisList, self).append(item)

    def extend(self, iterable):
        """
        iterates over an iterable and checks that each item is an instance of
        `RoboCompJoystickAdapter.AxisParams`. If it meets the condition, it extends
        the list by adding each item.

        Args:
            iterable (list): list of `RoboCompJoystickAdapter.AxisParams` objects
                that will be extended to the AxisList object.

        """
        for item in iterable:
            assert isinstance(item, RoboCompJoystickAdapter.AxisParams)
        super(AxisList, self).extend(iterable)

    def insert(self, index, item):
        """
        inserts an axis at a given index in the list and assigns it the provided
        parameter as its value.

        Args:
            index (int): 0-based index at which the new item will be inserted in
                the list.
            item (`RoboCompJoystickAdapter.AxisParams`.): axis parameter to be
                inserted into the list of AxisParams in the `insert()` method of
                the `RoboCompJoystickAdapter` class.
                
                		- `isinstance(item, RoboCompJoystickAdapter.AxisParams)`: Ensures
                that the input is an instance of the `AxisParams` class from the
                `RoboCompJoystickAdapter` module.
                

        """
        assert isinstance(item, RoboCompJoystickAdapter.AxisParams)
        super(AxisList, self).insert(index, item)

setattr(RoboCompJoystickAdapter, "AxisList", AxisList)
class ButtonsList(list):
    def __init__(self, iterable=list()):
        super(ButtonsList, self).__init__(iterable)

    def append(self, item):
        """
        adds a button parameter to the list of buttons maintained by the `ButtonsList`
        class, ensuring that the input is an instance of the `RoboCompJoystickAdapter.ButtonParams`
        class and calling the `super()` method to append it to the list maintained
        by the parent class.

        Args:
            item (`RoboCompJoystickAdapter.ButtonParams`.): button state parameters
                for RoboCompJoystickAdapter and is instance of ButtonsList class.
                
                		- `isinstance`: Ensures that the input is an instance of `RoboCompJoystickAdapter.ButtonParams`.
                		- `super`: Calls the `super` method to append the item to the
                parent class's list.
                
                	Properties of `item`:
                
                		- Type: The input is an instance of `RoboCompJoystickAdapter.ButtonParams`.
                		- Attributes: Depending on the implementation, this may include
                various properties related to the button, such as its label, state,
                or other relevant information.
                

        """
        assert isinstance(item, RoboCompJoystickAdapter.ButtonParams)
        super(ButtonsList, self).append(item)

    def extend(self, iterable):
        """
        iterates over an iterable and applies the assertion to each item. It then
        calls the parent class's `extend` method with the iterable as arguments.

        Args:
            iterable (list): sequence of `ButtonParams` objects to be added to the
                instance's `super().extend()` call.

        """
        for item in iterable:
            assert isinstance(item, RoboCompJoystickAdapter.ButtonParams)
        super(ButtonsList, self).extend(iterable)

    def insert(self, index, item):
        """
        insert an element at a specified index in the list of buttons managed by
        the object.

        Args:
            index (int): 0-based index of the position where the new button should
                be inserted in the list of buttons maintained by the `ButtonsList`
                object.
            item (`RoboCompJoystickAdapter.ButtonParams` instance.): ButtonParams
                object that specifies the button to be inserted into the list.
                
                		- Is an instance of `RoboCompJoystickAdapter.ButtonParams`
                		- Provides the button index and attributes such as state (pressed
                or not)
                

        """
        assert isinstance(item, RoboCompJoystickAdapter.ButtonParams)
        super(ButtonsList, self).insert(index, item)

setattr(RoboCompJoystickAdapter, "ButtonsList", ButtonsList)

import kinovaarmI
import joystickadapterI



class Publishes:
    def __init__(self, ice_connector, topic_manager):
        """
        initializes an instance of a class, setting attributes `ice_connector` and
        `topic_manager`.

        Args:
            ice_connector (`ICEConnector` object reference.): IceConnector object
                that is used to handle communication with the remote endpoints.
                
                		- `ice_connector`: A serialized object from an Interactive
                Connectivity Establishment (ICE) connector.
                
            topic_manager (`object`.): 3rd party manager that is responsible for
                creating, publishing, and managing the MQTT topics.
                
                		- `mprx`: The `mprx` dictionary stores the message processing
                rules for each topic.
                

        """
        self.ice_connector = ice_connector
        self.mprx={}
        self.topic_manager = topic_manager


    def create_topic(self, topic_name, ice_proxy):
        # Create a proxy to publish a AprilBasedLocalization topic
        """
        creates an Ice Proxy object to publish a `AprilBasedLocalization` topic,
        if it does not exist, it retrieves or creates the topic using the
        `TopicManager`. Finally, it sets the published proxy as the variable `mprx`
        for the given topic name.

        Args:
            topic_name (str): name of a topic to create or retrieve, and is used
                to identify the corresponding publisher or proxy object in the `topic_manager`.
            ice_proxy (`IcePy_oneway_ proxy` instance.): ice.oneway() method that
                creates a proxy for the published topic.
                
                		- `uncheckedCast`: This property of the `ice_proxy` object returns
                a published reference to a proxy topic.
                

        Returns:
            str: a proxy for publishing an AprilBasedLocalization topic.

        """
        topic = False
        try:
            topic = self.topic_manager.retrieve(topic_name)
        except:
            pass
        while not topic:
            try:
                topic = self.topic_manager.retrieve(topic_name)
            except IceStorm.NoSuchTopic:
                try:
                    topic = self.topic_manager.create(topic_name)
                except:
                    print(f'Another client created the {topic_name} topic? ...')
        pub = topic.getPublisher().ice_oneway()
        proxy = ice_proxy.uncheckedCast(pub)
        self.mprx[topic_name] = proxy
        return proxy

    def get_proxies_map(self):
        return self.mprx


class Requires:
    def __init__(self, ice_connector):
        """
        sets attributes to its instance attributes `self.ice_connector` and `self.mprx`.

        Args:
            ice_connector (object of type IceConnector.): icy interface connector
                in the initialization of the object.
                
                		- The attribute `ice_connector` contains a serialized object,
                which will be deconstructed by the Python serialization library
                (e.g., Pickle) at runtime.
                

        """
        self.ice_connector = ice_connector
        self.mprx={}

    def get_proxies_map(self):
        return self.mprx

    def create_proxy(self, property_name, ice_proxy):
        # Remote object connection for
        """
        creates a proxy connection to a remote object using Ice Python library.
        It takes in a property name and an ice proxy instance, then returns a tuple
        of True or False indicating success and the created proxy object respectively.

        Args:
            property_name (str): name of a property to be retrieved from the remote
                object through the Ice connection.
            ice_proxy (unchecked ice proxy object reference.): remote object to
                which the property value will be proxied.
                
                		- `uncheckedCast`: This attribute of `ice_proxy` allows casting
                an object without checking for validity, making it a flexible and
                safe choice when dealing with objects from unknown or untrusted sources.
                		- `mprx`: A property of the `ice_connector`, this refers to the
                mapping of properties in the remote object to their corresponding
                proxies in the local system.
                

        Returns:
            `(True, proxy)`, where ` True` indicates success and `proxy` represents
            the retrieved remote object proxy.: a tuple containing a boolean value
            and a proxy object for the requested property of a remote object.
            
            		- `True`: This indicates that the remote object connection was
            successful, and the `mprx` dictionary has been updated with the proxy
            object.
            		- `Proxy`: The `Proxy` attribute represents the created proxy object,
            which can be used to interact with the remote object.
            		- `None`: This value is returned when an error occurs during the
            remote object connection or when the `property_name` property does not
            exist in the Ice connector.
            

        """
        try:
            proxy_string = self.ice_connector.getProperties().getProperty(property_name)
            try:
                base_prx = self.ice_connector.stringToProxy(proxy_string)
                proxy = ice_proxy.uncheckedCast(base_prx)
                self.mprx[property_name] = proxy
                return True, proxy
            except Ice.Exception:
                print('Cannot connect to the remote object (CameraSimple)', proxy_string)
                # traceback.print_exc()
                return False, None
        except Ice.Exception as e:
            console.print_exception(e)
            console.log(f'Cannot get {property_name} property.')
            return False, None


class Subscribes:
    def __init__(self, ice_connector, topic_manager, default_handler):
        """
        initializes an instance of `JoystickHandler`, which creates an adapter for
        a Joystick topic and sets up the default handler for messages published
        to that topic.

        Args:
            ice_connector (`object`.): iced tea connection that allows the object
                to communicate with other Ice objects in the system.
                
                		- `ice_connector`: A connected ICE connector object.
                
                	Explanation: The `ice_connector` parameter represents a connected
                ICE (Interactive Connectivity Establishment) connector object,
                which is responsible for managing communication between the
                application and the network. It provides various properties and
                attributes that can be accessed and manipulated within the
                `TopicManager` class.
                
            topic_manager (`TopicManager` object.): topic manager that the adapter
                uses to manage topics related to its operation.
                
                		- `topic_manager`: This attribute is a TopicManager instance
                that represents the topics managed by this Joystick adapter.
                
                	Note: The `topic_manager` attribute is not destructured in the
                provided code snippet.
                
            default_handler (`joystickadapterI.JoystickAdapterI`.): default handler
                for events related to the `JoystickAdapterTopic`.
                
                		- `JoystickAdapterTopic`: The topic managed by this class's instance.
                		- `JoystickAdapterI`: A reference to an instance of `joystickadapteri.JoystickAdapterI`.
                
                	In summary, the `default_handler` parameter represents a managed
                topic and an associated `JoystickAdapterI` instance that can be
                used to handle messages on that topic.
                

        """
        self.ice_connector = ice_connector
        self.topic_manager = topic_manager

        self.JoystickAdapter = self.create_adapter("JoystickAdapterTopic", joystickadapterI.JoystickAdapterI(default_handler))

    def create_adapter(self, property_name, interface_handler):
        """
        creates an adapter object and adds a proxy to it. It also registers a
        handler for the specified property name and subscribe to the associated
        topic, all in a loop until the topic is created or the maximum number of
        attempts has been reached.

        Args:
            property_name (str): name of the topic that will be created or subscribed
                to by the adapter.
            interface_handler (ice.IOReturn.): handler for the interface that is
                being created and added to the adapter.
                
                		- `interface_handler`: A handle to an Iceoryx interface handler.
                
                	The adapter created by this function has various attributes:
                
                		- `adapter`: The newly created adapter object.
                		- `proxy`: The proxy instance returned by the `addWithUUID`
                method of the adapter.
                		- `topic_name`: The name of the topic to which the adapter is subscribed.
                		- `qos`: The quality-of-service settings for the subscription,
                contained in a dictionary.
                

        Returns:
            `Ice::Adapter`.: an activated IceoryKube adapter instance.
            
            		- `adapter`: The created object adapter instance.
            		- `proxy`: The proxy instance added to the adapter.
            		- `qos`: The QoS (Quality of Service) parameters for the subscription.
            		- `topic_name`: The name of the topic being subscribed to.
            
            	In addition, the function takes two input arguments:
            
            		- `property_name`: The property name to be used as the adapter's
            activation hook.
            		- `interface_handler`: An interface handle for the adapter's activation.
            

        """
        adapter = self.ice_connector.createObjectAdapter(property_name)
        handler = interface_handler
        proxy = adapter.addWithUUID(handler).ice_oneway()
        topic_name = property_name.replace('Topic','')
        subscribe_done = False
        while not subscribe_done:
            try:
                topic = self.topic_manager.retrieve(topic_name)
                subscribe_done = True
            except Ice.Exception as e:
                console.log("Error. Topic does not exist (creating)", style="blue")
                time.sleep(1)
                try:
                    topic = self.topic_manager.create(topic_name)
                    subscribe_done = True
                except:
                    console.log(f"Error. Topic {Text(topic_name, style='red')} could not be created. Exiting")
                    status = 0
        qos = {}
        topic.subscribeAndGetPublisher(qos, proxy)
        adapter.activate()
        return adapter


class Implements:
    def __init__(self, ice_connector, default_handler):
        """
        creates an instance of a class by calling the `create_adapter` method and
        setting its attributes accordingly.

        Args:
            ice_connector (object/instance.): Ice connector for the Kinova arm.
                
                		- `ice_connector`: A deserialized input containing information
                about the Ice connector, such as its type and attributes.
                
            default_handler (`kinovaarmI.KinovaArmI`.): default handler for the
                `KinovaArm` instance, which is created when the function is called
                without providing any specific implementation.
                
                		- `kinovaarmI`: This is an instance of the `KinovaArmI` class,
                which has various attributes and methods related to controlling a
                Kinova arm. These include `kinova_host`, `timeout`, `log_level`,
                and `connect`.
                

        """
        self.ice_connector = ice_connector
        self.kinovaarm = self.create_adapter("KinovaArm", kinovaarmI.KinovaArmI(default_handler))

    def create_adapter(self, property_name, interface_handler):
        """
        creates an adapter object for a given property name and interface handler.
        It adds the interface handler to the adapter and activates it.

        Args:
            property_name (str): name of a property to be created as an adapter
                in the function, which is then used to generate the actual adapter
                object.
            interface_handler (str): 3A (adapter) interface to be handled by the
                `createAdapter()` method.

        """
        adapter = self.ice_connector.createObjectAdapter(property_name)
        adapter.add(interface_handler, self.ice_connector.stringToIdentity(property_name.lower()))
        adapter.activate()


class InterfaceManager:
    def __init__(self, ice_config_file):
        # TODO: Make ice connector singleton
        """
        initializes an Ice connector singleton and sets up topic manager, requires,
        publishes, implements and subscribes properties based on the configuration
        file.

        Args:
            ice_config_file (str): configuration file for Ice, which is used to
                initialize the Ice connector and set up its properties and topics.

        """
        self.ice_config_file = ice_config_file
        self.ice_connector = Ice.initialize(self.ice_config_file)
        needs_rcnode = True
        self.topic_manager = self.init_topic_manager() if needs_rcnode else None

        self.status = 0
        self.parameters = {}
        for i in self.ice_connector.getProperties():
            self.parameters[str(i)] = str(self.ice_connector.getProperties().getProperty(i))
        self.requires = Requires(self.ice_connector)
        self.publishes = Publishes(self.ice_connector, self.topic_manager)
        self.implements = None
        self.subscribes = None



    def init_topic_manager(self):
        # Topic Manager
        """
        initiates an IceStorm Topic Manager proxy and returns it, which allows
        communication with the RCnode service via a Pub/Sub interface.

        Returns:
            `IceStorm.TopicManagerPrx` instance.: an instance of the
            `IceStorm.TopicManagerPrx` object.
            
            		- `IceStorm.TopicManagerPrx`: This is an object pointer returned by
            the `getProperties().getProperty("TopicManager.Proxy")` method, which
            represents a proxy object that provides access to the Topic Manager service.
            		- `obj`: This is the actual object pointer that is returned by the
            `getProperties()` method and converted to a proxy object using the
            `stringToProxy()` method.
            

        """
        proxy = self.ice_connector.getProperties().getProperty("TopicManager.Proxy")
        obj = self.ice_connector.stringToProxy(proxy)
        try:
            return IceStorm.TopicManagerPrx.checkedCast(obj)
        except Ice.ConnectionRefusedException as e:
            console.log(Text('Cannot connect to rcnode! This must be running to use pub/sub.', 'red'))
            exit(-1)

    def set_default_hanlder(self, handler):
        """
        sets the default ice handler and subscribes to a topic manager for a given
        ice connector.

        Args:
            handler (object.): ice_controller interface which is implemented by
                the passed handler object.
                
                	1/ `implements`: The `Implements` attribute is added to the
                object's members, representing the handler as an implementor of
                the IceConnector interface.
                	2/ `subscribes`: The `Subscribes` attribute is added to the
                object's members, representing the handler as a subscriber of the
                topic manager and any related topics.
                

        """
        self.implements = Implements(self.ice_connector, handler)
        self.subscribes = Subscribes(self.ice_connector, self.topic_manager, handler)

    def get_proxies_map(self):
        """
        updates a dictionary with proxies information from `requires` and `publishes`
        attributes, and returns the updated map.

        Returns:
            dict: a dictionary containing the union of the proxies maps of its
            parents and children.

        """
        result = {}
        result.update(self.requires.get_proxies_map())
        result.update(self.publishes.get_proxies_map())
        return result

    def destroy(self):
        """
        terminates an iceConnector object's existence, if it exists.

        """
        if self.ice_connector:
            self.ice_connector.destroy()




