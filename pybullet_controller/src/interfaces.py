import time
import Ice
import IceStorm
from rich.console import Console, Text
console = Console()


Ice.loadSlice("-I ./src/ --all ./src/CameraRGBDSimple.ice")
import RoboCompCameraRGBDSimple
Ice.loadSlice("-I ./src/ --all ./src/JoystickAdapter.ice")
import RoboCompJoystickAdapter
Ice.loadSlice("-I ./src/ --all ./src/KinovaArm.ice")
import RoboCompKinovaArm

class ImgType(list):
    def __init__(self, iterable=list()):
        super(ImgType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, byte)
        super(ImgType, self).append(item)

    def extend(self, iterable):
        """
        Ensures that all elements passed to it are of type `byte`, then it extends
        the `ImgType` object with the provided iterable of elements.

        Args:
            iterable (`byte`.): iterable of bytes that will be processed and added
                to the `ImgType` instance.
                
                	* `isinstance(item, byte)` - checks if each item in the iterable
                is an instance of the `byte` type.
                	* `super(ImgType, self).extend(iterable)` - extends the object
                with the deserialized items from the iterable using the `super()`
                method.

        """
        for item in iterable:
            assert isinstance(item, byte)
        super(ImgType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, byte)
        super(ImgType, self).insert(index, item)

setattr(RoboCompCameraRGBDSimple, "ImgType", ImgType)
class DepthType(list):
    def __init__(self, iterable=list()):
        super(DepthType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, byte)
        super(DepthType, self).append(item)

    def extend(self, iterable):
        """
        Ensures that all items passed to it are of type `byte`, and then calls the
        `super`.

        Args:
            iterable (`byte`.): 2D array of bytes that are to be processed and
                checked for depth channel type.
                
                	* All items in `iterable` are of type `bytes`.
                	* The type of each item is validated using the `assert` statement.

        """
        for item in iterable:
            assert isinstance(item, byte)
        super(DepthType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, byte)
        super(DepthType, self).insert(index, item)

setattr(RoboCompCameraRGBDSimple, "DepthType", DepthType)
class PointsType(list):
    def __init__(self, iterable=list()):
        super(PointsType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompCameraRGBDSimple.Point3D)
        super(PointsType, self).append(item)

    def extend(self, iterable):
        """
        Adds an iterable of `RoboCompCameraRGBDSimple.Point3D` objects to the
        current points set of a class called `PointsType`.

        Args:
            iterable (int): 3D points to be added to the `PointsType` object through
                its extension method.

        """
        for item in iterable:
            assert isinstance(item, RoboCompCameraRGBDSimple.Point3D)
        super(PointsType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompCameraRGBDSimple.Point3D)
        super(PointsType, self).insert(index, item)

setattr(RoboCompCameraRGBDSimple, "PointsType", PointsType)
class AxisList(list):
    def __init__(self, iterable=list()):
        super(AxisList, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompJoystickAdapter.AxisParams)
        super(AxisList, self).append(item)

    def extend(self, iterable):
        """
        Takes an iterable object and extends the AxisList by adding each element
        as a RoboCompJoystickAdapter.AxisParams instance.

        Args:
            iterable (list): iterable of `RoboCompJoystickAdapter.AxisParams`
                objects to be extended with the corresponding axes for the joystick
                adapter.

        """
        for item in iterable:
            assert isinstance(item, RoboCompJoystickAdapter.AxisParams)
        super(AxisList, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompJoystickAdapter.AxisParams)
        super(AxisList, self).insert(index, item)

setattr(RoboCompJoystickAdapter, "AxisList", AxisList)
class ButtonsList(list):
    def __init__(self, iterable=list()):
        super(ButtonsList, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompJoystickAdapter.ButtonParams)
        super(ButtonsList, self).append(item)

    def extend(self, iterable):
        """
        Adds elements to a list object by iterating over an iterable and asserting
        that each element is of type `RoboCompJoystickAdapter.ButtonParams`.

        Args:
            iterable (tuple): sequence of `RoboCompJoystickAdapter.ButtonParams`
                objects to be extended by the `ButtonsList` class.

        """
        for item in iterable:
            assert isinstance(item, RoboCompJoystickAdapter.ButtonParams)
        super(ButtonsList, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompJoystickAdapter.ButtonParams)
        super(ButtonsList, self).insert(index, item)

setattr(RoboCompJoystickAdapter, "ButtonsList", ButtonsList)
class TJointSeq(list):
    def __init__(self, iterable=list()):
        super(TJointSeq, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompKinovaArm.TJoint)
        super(TJointSeq, self).append(item)

    def extend(self, iterable):
        """
        Iterates over an iterable object and checks each item to ensure it is a
        `RoboCompKinovaArm.TJoint`. If it passes the check, it adds it to the sequence.

        Args:
            iterable (list): list of `RoboCompKinovaArm.TJoint` objects to be added
                to the `TJointSeq` object through the `extend()` method.

        """
        for item in iterable:
            assert isinstance(item, RoboCompKinovaArm.TJoint)
        super(TJointSeq, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompKinovaArm.TJoint)
        super(TJointSeq, self).insert(index, item)

setattr(RoboCompKinovaArm, "TJointSeq", TJointSeq)
class Speeds(list):
    def __init__(self, iterable=list()):
        super(Speeds, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, float)
        super(Speeds, self).append(item)

    def extend(self, iterable):
        """
        Adds elements to a sequence in passive voice without repeating the question
        or using any personal statements. The provided function checks each element's
        type and ensures it is an instance of float before adding it to the sequence.

        Args:
            iterable (float): sequence of floating-point values that are to be
                processed by the `Speeds` class.

        """
        for item in iterable:
            assert isinstance(item, float)
        super(Speeds, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, float)
        super(Speeds, self).insert(index, item)

setattr(RoboCompKinovaArm, "Speeds", Speeds)
class Angles(list):
    def __init__(self, iterable=list()):
        super(Angles, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, float)
        super(Angles, self).append(item)

    def extend(self, iterable):
        """
        Is called with an iterable containing only instances of type `float`. It
        checks each item in the iterable and asserts that it is a float, then adds
        it to the Angles object's internal list.

        Args:
            iterable (float): 3D angles that are to be extended by the `super()`
                call, consisting of float values.

        """
        for item in iterable:
            assert isinstance(item, float)
        super(Angles, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, float)
        super(Angles, self).insert(index, item)

setattr(RoboCompKinovaArm, "Angles", Angles)

import joystickadapterI



class Publishes:
    def __init__(self, ice_connector, topic_manager):
        """
        Sets attributes `ice_connector` and `topic_manager` for an object, while
        creating a new instance of the class.

        Args:
            ice_connector (`ice_connector`.): icy interface connector for the
                current process.
                
                	* `ice_connector`: This is a ` IceConnector` instance, which
                represents a connection to an Interoperability Crossing (Ice) device.
                
                	The `self.mprx` dictionary stores mapping details for the
                `ice_connector`, while the `self.topic_manager` refers to a
                `TopicManager` object, managing topics related to the `ice_connector`.
            topic_manager (`TopicManager` object.): topic manager that manages the
                topics and their corresponding data models for the ice connector.
                
                	* `self.mprx`: An empty dictionary representing the manager's PRX.
                	* `self.topic_manager`: A class instance representing the Topic
                Manager.

        """
        self.ice_connector = ice_connector
        self.mprx={}
        self.topic_manager = topic_manager


    def create_topic(self, topic_name, ice_proxy):
        # Create a proxy to publish a AprilBasedLocalization topic
        """
        Creates a new topic or retrieves an existing one based on its name, and
        returns a publisher object that can be used to publish messages to the topic.

        Args:
            topic_name (str): name of a topic that is retrieved or created in the
                function.
            ice_proxy (`ice.Publisher` object.): iced proxy object that is generated
                by casting the `getPublisher().ice_oneway()` IceStorm publisher
                to an unchecked ice proxy object.
                
                	* `uncheckedCast`: This is an ice_oneway method that takes the
                published object as an argument and returns its ice proxy.

        Returns:
            undefined: an Ice proxy object referencing a published topic's publisher.
            
            	* `pub`: A reference to the publisher object, which is an `Ice.oneway()`
            proxy.
            	* `proxy`: The result of the unchecked cast of the `pub` object to a
            `IceProxy`.

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
        Sets up instance variables `ice_connector`, `mprx`, and three proxies:
        `CameraRGBDSimpleProxy`, `KinovaArmProxy`, and `RoboCompCameraRGBDSimplePrx`.

        Args:
            ice_connector ("ProxyReference".): ICE (Interoperability Cross-Domain
                Bridge) connector, which is used to enable communication between
                different Robot Operating System (ROS) nodes and components.
                
                	* `self.ice_connector`: The instance of the IceConnector class.
                	* `self.mprx`: A dictionary of the proxy's members (properties
                or attributes).

        """
        self.ice_connector = ice_connector
        self.mprx={}

        self.CameraRGBDSimple = self.create_proxy("CameraRGBDSimpleProxy", RoboCompCameraRGBDSimple.CameraRGBDSimplePrx)

        self.KinovaArm = self.create_proxy("KinovaArmProxy", RoboCompKinovaArm.KinovaArmPrx)

    def get_proxies_map(self):
        return self.mprx

    def create_proxy(self, property_name, ice_proxy):
        # Remote object connection for
        """
        Creates a proxy for a remote object using the `Ice.Communicator` class and
        returns a tuple of the created proxy and a boolean value indicating whether
        the operation was successful.

        Args:
            property_name (str): name of the property to be retrieved from the
                remote object.
            ice_proxy (IceProxy object.): iced proxy object that will be used to
                convert the remote object's property into a Python proxy object.
                
                	* `uncheckedCast`: This attribute is used to convert an object
                into a proxy type. It returns the converted object or raises an
                exception if it cannot be cast.
                	* `mprx`: A dictionary-like object that maps property names to
                proxies. Each key in the dictionary is a property name, and the
                value is a proxy object for that property.

        Returns:
            undefined: a tuple containing either a valid proxy object or an error
            message.
            
            	* `True`: This is a boolean value indicating whether the proxy was
            successfully created.
            	* `proxy`: This is the created Ice proxy object, which represents the
            remote CameraSimple object. It can be used to call methods on the
            remote object and to receive method invocations.
            
            	In the except block, if an exception occurs during the creation of
            the proxy, the output is:
            
            	* `False`: This is a boolean value indicating whether the proxy was
            created successfully.
            	* `None`: This is the error message that occurred during the creation
            of the proxy.

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
        Sets up a topic manager and creates an adapter for a JoystickTopic based
        on the `JoystickAdapterI` class.

        Args:
            ice_connector (`ICEConnector`.): ICE (Intermediate Capability Exchange)
                connector, which is used to manage and exchange information between
                components in the system.
                
                	* `self.ice_connector`: This attribute holds an instance of the
                `IceConnector` class, which serves as the central point for accessing
                and managing Neos' features through ROS messages. It provides
                methods to communicate with Neos, including subscribing to and
                publishing topics, as well as handling messages from Neos.
                	* `self.topic_manager`: This attribute holds an instance of the
                `TopicManager` class, which is responsible for managing the topic
                trees and subscriptions for a specific Neos system. It provides
                methods to subscribe to topics, unsubscribe from topics, and manage
                the lifecycle of topics.
                	* `self.JoystickAdapter`: This attribute holds an instance of the
                `JoystickAdapter` class, which provides a convenient way to access
                the joystick input from ROS messages. It offers methods to subscribe
                to joystick-related topics and publish updates to the joystick state.
            topic_manager (`TopicManager`.): ical manager to which the `JoystickAdapter`
                instance will be subscribed to and publish to.
                
                	* `topic_manager`: This is an instance of `TopicManagerI`, which
                manages topics related to various applications or services. It has
                various attributes such as `topics`, `topic_type`, `client`, and
                others, which can be used to configure and interact with the topics
                managed by this object.
            default_handler (`JoystickAdapterI` object.): default handler for
                handling events and messages published to the JoystickAdapterTopic
                topic.
                
                	* `JoystickAdapterTopic`: This is the topic name used to communicate
                with the joystick adapter.

        """
        self.ice_connector = ice_connector
        self.topic_manager = topic_manager

        self.JoystickAdapter = self.create_adapter("JoystickAdapterTopic", joystickadapterI.JoystickAdapterI(default_handler))

    def create_adapter(self, property_name, interface_handler):
        """
        Creates an IceConnector object adapter and adds an interface handler to
        it using Ice's oneway() method, then retrieves or creates a topic based
        on its name and subscribes to it using the adapter's activate() method.

        Args:
            property_name (str): name of a topic to which the iceoryx adapter
                should subscribe.
            interface_handler (int): icy handler for the topic being subscribed
                to, and is used by the function to create a Proxy object that will
                receive updates from the topic.

        Returns:
            undefined: an Ice::Adapter object that can be used to subscribe to a
            topic.
            
            	* `adapter`: An instance of the `Adapter` class, which represents a
            connection to a remote object adapter.
            	* `handler`: An instance of the `InterfaceHandler` class, which handles
            interface calls on behalf of the adapter.
            	* `proxy`: A reference to the proxy object that was created by adding
            an ice_oneway() method to the adapter.
            	* `topic_name`: The name of the topic for which a subscription is
            being created.
            	* `qos`: A dictionary of quality of service (QoS) properties, which
            are used to specify the QoS settings for the subscription.
            	* `status`: An integer value that indicates the status of the function
            call, with 0 indicating success and any other value indicating an error
            or failure.

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
        self.ice_connector = ice_connector

    def create_adapter(self, property_name, interface_handler):
        """
        Creates an ObjectAdapter using the `self.ice_connector.createObjectAdapter()`
        method, and then adds an interface handler to the adapter using the `add()`
        method. Finally, it activates the adapter using the `activate()` method.

        Args:
            property_name (str): name of the property that the `interface_handler`
                is associated with, which is used to create an adapter object and
                add the `interface_handler` to it.
            interface_handler (`Identity`.): 3GPP TLV-based data object handle of
                an interface to be activated by the `createObjectAdapter()` method
                call.
                
                	* `self.ice_connector`: This is an instance of the `IceConnector`
                class, which provides methods for interacting with IceBERT.
                	* `property_name`: This is a string variable that represents the
                name of the property being serialized or deserialized.
                	* `stringToIdentity()`: This method converts a given string to
                an identity object, which can be used as a reference in IceBERT's
                identity map.

        """
        adapter = self.ice_connector.createObjectAdapter(property_name)
        adapter.add(interface_handler, self.ice_connector.stringToIdentity(property_name.lower()))
        adapter.activate()


class InterfaceManager:
    def __init__(self, ice_config_file):
        # TODO: Make ice connector singleton
        """
        Initializes an Ice client configuration, loads configuration file
        `ice_config_file`, creates an Ice client instance `self.ice_connector`,
        and sets various parameters and objects based on the loaded configuration.

        Args:
            ice_config_file (file path/name.): icy configuration file for the given
                class, which is used to initialize the Ice framework and retrieve
                information about the class's properties, requirements, publications,
                subscriptions, and other attributes.
                
                	* `self.ice_config_file`: The file containing the Ice configuration.
                	* `needs_rcnode`: A boolean indicating whether an RCNode needs
                to be initialized (True) or not (False).
                	* `topic_manager`: An object of type `TopicManager` that manages
                the topics in the Ice environment. It is initialized only if
                `needs_rcnode` is True.
                	* `status`: An integer status code indicating whether the Ice
                configuration file is valid or not.
                	* `parameters`: A dictionary containing the property values of
                the Ice configuration file. Each key is a property name, and each
                value is the corresponding property value.
                	* `requires`: An instance of the `Requires` class, which represents
                the dependencies between the services in the Ice environment.
                	* `publishes`: An instance of the `Publishes` class, which
                represents the publish relationships between the services in the
                Ice environment.
                	* `implements`: None or an empty list, indicating that no services
                are implemented in this class.
                	* `subscribes`: None or an empty list, indicating that no services
                are subscribed to in this class.

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
        Converts a string representation of a proxy object into an actual IceStorm
        TopicManager object and returns it.

        Returns:
            undefined: an IceStorm TopicManager Prx object.
            
            	* `proxy`: The Proxy object representing the TopicManager proxy.
            	* `obj`: The initialized TopicManagerPrx object.

        """
        proxy = self.ice_connector.getProperties().getProperty("TopicManager.Proxy")
        obj = self.ice_connector.stringToProxy(proxy)
        try:
            return IceStorm.TopicManagerPrx.checkedCast(obj)
        except Ice.ConnectionRefusedException as e:
            console.log(Text('Cannot connect to rcnode! This must be running to use pub/sub.', 'red'))
            exit(-1)

    def set_default_hanlder(self, handler):
        self.implements = Implements(self.ice_connector, handler)
        self.subscribes = Subscribes(self.ice_connector, self.topic_manager, handler)

    def get_proxies_map(self):
        """
        Updates a result dictionary with the proxies maps for both required and
        published proxies, returning the updated result dictionary.

        Returns:
            undefined: a dictionary containing the union of the `requires` and
            `publishes` proxies.
            
            	* `result`: The overall output of the function, which contains both
            the proxies provided by the `requires` and `publishes` collections.
            	* `update()`: This method is used to update the result object with
            the contents of either the `requires` or `publishes` collection's
            `get_proxies_map()` function call.
            
            	The `result` object may have multiple properties, such as `requires`
            and `publishes`, each representing the proxies provided by one of these
            collections. The order of the keys in the result object corresponds
            to the order in which the functions were called, with any proxy from
            the `requires` collection appearing before those from the `publishes`
            collection.

        """
        result = {}
        result.update(self.requires.get_proxies_map())
        result.update(self.publishes.get_proxies_map())
        return result

    def destroy(self):
        if self.ice_connector:
            self.ice_connector.destroy()




