import time
import Ice
import IceStorm
from rich.console import Console, Text
console = Console()


Ice.loadSlice("-I ./src/ --all ./src/Contactile.ice")
import RoboCompContactile
Ice.loadSlice("-I ./src/ --all ./src/KinovaArm.ice")
import RoboCompKinovaArm

class TJointSeq(list):
    """
    Manages a sequence of `RoboCompKinovaArm.TJoint` objects, providing methods
    for appending, extending, and inserting joints.

    """
    def __init__(self, iterable=list()):
        super(TJointSeq, self).__init__(iterable)

    def append(self, item):
        """
        Adds an item to the sequence, ensuring it is an instance of the
        RoboCompKinovaArm.TJoint class before appending it to the parent list.

        Args:
            item (RoboCompKinovaArm.TJoint): Asserted to be an instance of that
                class before being appended to the sequence.

        """
        assert isinstance(item, RoboCompKinovaArm.TJoint)
        super(TJointSeq, self).append(item)

    def extend(self, iterable):
        """
        Iterates over an input iterable and asserts that each item is an instance
        of the RoboCompKinovaArm.TJoint class before extending the sequence with
        those items using the superclass's `extend` method.

        Args:
            iterable (Sequence[TJoint]): An iterable collection of TJoint objects
                to be extended.

        """
        for item in iterable:
            assert isinstance(item, RoboCompKinovaArm.TJoint)
        super(TJointSeq, self).extend(iterable)

    def insert(self, index, item):
        """
        Modifies the list's index and item by inserting an instance of the
        RoboCompKinovaArm.TJoint object at the specified index.

        Args:
            index (int): Used to specify the position at which the new joint should
                be inserted in the sequence.
            item (RoboCompKinovaArm.TJoint): Asserted to be an instance of the
                TJoint class before calling the superclass's insert method.

        """
        assert isinstance(item, RoboCompKinovaArm.TJoint)
        super(TJointSeq, self).insert(index, item)

setattr(RoboCompKinovaArm, "TJointSeq", TJointSeq)
class Speeds(list):
    """
    Manages a list of floating-point numbers, providing methods to append, extend,
    and insert elements with type checks for input values.

    """
    def __init__(self, iterable=list()):
        super(Speeds, self).__init__(iterable)

    def append(self, item):
        """
        Adds an item to the list, checking that it is of type float before appending
        it to the superclass list.

        Args:
            item (float): Asserted to be of that type before being appended.

        """
        assert isinstance(item, float)
        super(Speeds, self).append(item)

    def extend(self, iterable):
        """
        Takes an iterable and ensures that each item is an instance of `float`.
        Then, it calls the `super().extend()` method to add the items to the Speeds
        list.

        Args:
            iterable (Iterable[float]): An iterable object containing floating-point
                numbers that the Speeds class will validate and add to its internal
                list.

        """
        for item in iterable:
            assert isinstance(item, float)
        super(Speeds, self).extend(iterable)

    def insert(self, index, item):
        """
        Inserts an item at a given index, checking that the item is a float and
        calling the parent class's `insert` method with the index and item as arguments.

        Args:
            index (int): Used to indicate the position at which the new element
                should be inserted.
            item (float): Asserted to be an instance of that type before it is
                passed to the superclass's insert method.

        """
        assert isinstance(item, float)
        super(Speeds, self).insert(index, item)

setattr(RoboCompKinovaArm, "Speeds", Speeds)
class Angles(list):
    """
    Is a list-based implementation of angles, providing methods for appending,
    extending, and inserting angles as floating-point values.

    """
    def __init__(self, iterable=list()):
        super(Angles, self).__init__(iterable)

    def append(self, item):
        """
        Adds a float value to the list of angles.

        Args:
            item (float): Asserted to be an instance of that type before being
                appended to the Angles object.

        """
        assert isinstance(item, float)
        super(Angles, self).append(item)

    def extend(self, iterable):
        """
        Checks if each item in the iterable is a float, then calls the parent
        class's `extend` method with the same iterable.

        Args:
            iterable (Iterable[float]): Used to add elements to the Angles object.

        """
        for item in iterable:
            assert isinstance(item, float)
        super(Angles, self).extend(iterable)

    def insert(self, index, item):
        """
        Inserts an item into the list at a specified index, checking that the input
        item is a float and calling the superclass's implementation before inserting
        it.

        Args:
            index (int): Used to specify the position where the new element will
                be inserted in the list.
            item (float): Asserted to be an instance of that type before being
                passed to the superclass method for insertion.

        """
        assert isinstance(item, float)
        super(Angles, self).insert(index, item)

setattr(RoboCompKinovaArm, "Angles", Angles)

import kinovaarmI



class Publishes:
    """
    Manages topics and provides a way to publish messages to them through a proxy
    object, allowing multiple clients to create and manage topics independently.

    Attributes:
        ice_connector (object): A reference to the Ice Connector used for communication
            with the server.
        mprx (Dict[str,IceObject]): A mapping of topics to their corresponding Ice
            proxies.
        topic_manager (TopicManager|IceStormNoSuchTopic): Responsible for managing
            topics created by the instance.

    """
    def __init__(self, ice_connector, topic_manager):
        """
        Initializes instance variables: ice_connector and topic_manager, which are
        used to manage communication between the publisher and the Ice platform.

        Args:
            ice_connector (object): Used to store an instance of `IceConnector`,
                which provides connectivity between distributed components.
            topic_manager (object): Used to store an instance of `TopicManager`.

        """
        self.ice_connector = ice_connector
        self.mprx={}
        self.topic_manager = topic_manager


    def create_topic(self, topic_name, ice_proxy):
        # Create a proxy to publish a AprilBasedLocalization topic
        """
        Creates a new topic or retrieves an existing one based on its name, and
        returns an Ice proxy for the publisher of that topic.

        Args:
            topic_name (str): Used to represent the name of a topic to be created
                or retrieved from the Topic Manager.
            ice_proxy (IceStorm.Publisher.UncheckedCast[Any] ): Used to convert
                the publisher into an unchecked cast proxy.

        Returns:
            IcePyUncheckedCastProxy: An instance of the IcePyUncheckedCast class,
            representing a one-way publisher for the specified topic name.

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
    """
    Manages proxy objects for accessing remote objects through an Ice connector.
    It creates and stores proxies for various properties, allowing for easy access
    to the remote objects through a consistent interface.

    Attributes:
        ice_connector (IceConnector|None): Used to manage communication between
            objects across different partitions in a distributed system.
        mprx (Dict[str,ice_proxy]): Used to store proxy objects for different
            properties of a remote object.
        Contactile (RoboCompContactileContactilePrx|IceObject): Used to create a
            proxy for the Contactile object.
        create_proxy (bool|IcePrx): Used to create a proxy object for a remote
            object property.

    """
    def __init__(self, ice_connector):
        """
        Initializes instance variables ice_connector and mprx, as well as creating
        a proxy for the Contactile object using the create_proxy method.

        Args:
            ice_connector (RoboCompICEConnectorPrx): Required to initialize the
                object. It represents an instance of the RoboCompICEConnector
                interface, which provides access to the RoboComp toolkit's
                functionality for robot control and interaction with external systems.

        """
        self.ice_connector = ice_connector
        self.mprx={}

        self.Contactile = self.create_proxy("ContactileProxy", RoboCompContactile.ContactilePrx)

    def get_proxies_map(self):
        return self.mprx

    def create_proxy(self, property_name, ice_proxy):
        # Remote object connection for
        """
        Creates a proxy object for a specified property of an IceGrid connection.

        Args:
            property_name (str): Used to specify the name of the property in the
                remote object to be accessed through the proxy.
            ice_proxy (Ice.Proxy | NoneType): Used to create a proxy object for
                the remote object.

        Returns:
            bool|str: 2-element tuple containing the success status and the created
            proxy object.

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
    """
    Creates an ObjectAdapter and adds a Proxy to it. It then subscribes to a topic,
    retrieves its publisher, and activates the adapter.

    Attributes:
        ice_connector (ObjectAdapter|IceReference): Used to create an Ice reference
            for connecting to the Intermediate Java Representation (IJR) server.
        topic_manager (TopicManager|IceObjectPrx): Used to manage topics in the system.

    """
    def __init__(self, ice_connector, topic_manager, default_handler):
        """
        Sets up instance variables ice_connector, topic_manager, and default_handler
        for further use by the class.

        Args:
            ice_connector (object): Used to set the IceConnector instance for the
                TopicManager class.
            topic_manager (object): Used to manage topics related to the IceConnector
                instance.
            default_handler (Callable[object, object]): Used to define a default
                handler for topics that are not handled by any other handler in
                the system.

        """
        self.ice_connector = ice_connector
        self.topic_manager = topic_manager

    def create_adapter(self, property_name, interface_handler):
        """
        Creates an object adapter and adds a proxy to it, then subscribes to a
        topic using a specified QoS and activates the adapter.

        Args:
            property_name (str | PropertyName): Used to specify the name of the
                topic for which an adapter is to be created.
            interface_handler (object): Required to handle the interface for the
                created adapter.

        Returns:
            Adapter: An iceoryteam::adapter object that represents a new adapter
            instance created through Iceoryteam's createObjectAdapter method.

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
    """
    Defines an adapter creator that enables communication between Ice houses and
    Kinova arms by creating adapters for property names and interface handlers.

    Attributes:
        ice_connector (iceConnector|iceProperties): Used to manage objects created
            by the adapter.
        kinovaarm (KinovaArmI): Created by calling the `create_adapter` method
            with the property name "KinovaArm".
        create_adapter (Optional[Adapter]): Used to create a new adapter instance
            with given name and interface handler.

    """
    def __init__(self, ice_connector, default_handler):
        """
        Initializes the object by setting its ice connector and creating an adapter
        for the KinovaArm device using the create_adapter() method.

        Args:
            ice_connector (object): Used to store the instance of an IceConnector
                class, which is responsible for handling communication between the
                application and the Kinova Arm device.
            default_handler (KinovaArmI): Passed to create an instance of the
                KinovaArm class, providing a default handler for the arm's actions.

        """
        self.ice_connector = ice_connector
        self.kinovaarm = self.create_adapter("KinovaArm", kinovaarmI.KinovaArmI(default_handler))

    def create_adapter(self, property_name, interface_handler):
        """
        Creates an adapter object using the `ice_connector.createObjectAdapter`
        method and adds an interface handler to it using the `add` method. The
        added interface handler is then activated using the `activate` method.

        Args:
            property_name (str | PropertyNameType): Used to specify the name of
                the property for which an adapter will be created.
            interface_handler (Any | Ice.Reference): Used to specify the interface
                that the adapter will handle.

        """
        adapter = self.ice_connector.createObjectAdapter(property_name)
        adapter.add(interface_handler, self.ice_connector.stringToIdentity(property_name.lower()))
        adapter.activate()


class InterfaceManager:
    """
    Manages interfaces, including setting default handlers for implementations and
    subscriptions, getting proxies maps, and destroying the interface when needed.

    Attributes:
        ice_config_file (File|str): Used to store the configuration file for the
            Interprise Communication Interface (Ice) connection.
        ice_connector (Iceinitializeselfice_config_file): Used to initialize the
            Ice connection and retrieve properties.
        topic_manager (IceStormTopicManagerPrx|IceStormTopicManager): Initialized
            when the object is created through the `init_topic_manager()` method.
            It is used to manage topics in the interface.
        init_topic_manager (IceStormTopicManagerPrx|IceConnectionRefusedException):
            Responsible for initializing a TopicManager proxy using the configuration
            file.
        status (int): 0 by default, indicating a successful initialization of the
            interface manager.
        parameters (Dict[str,str]): Created by iterating over the properties
            returned by the Ice connector's `getProperties()` method, storing their
            values as strings.
        requires (Requires): Used to store a map of proxy identifiers to their
            corresponding IceStorm proxies.
        publishes (PublishesPrx|InterfaceManager): Used to manage publish operations
            by mapping interfaces to their corresponding pub/sub topics.
        implements (Implements|Proxy): Used to specify the default handler for
            implementing interfaces.
        subscribes (Tuple[str,IceStormTopicManagerPrx]): Used to store the
            subscriptions managed by the interface manager.

    """
    def __init__(self, ice_config_file):
        # TODO: Make ice connector singleton
        """
        Initializes instance variables and sets up the Ice framework, topic manager,
        and properties.

        Args:
            ice_config_file (str | Path): Used to initialize Ice components from
                an Ice configuration file.

        """
        self.ice_config_file = ice_config_file
        self.ice_connector = Ice.initialize(self.ice_config_file)
        needs_rcnode = False
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
        Initates and manages a Topic Manager instance for communication with a
        remote node.

        Returns:
            IceStormTopicManagerPrx: A proxy object for the TopicManager interface.

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
        Sets the default handler for incoming messages on the interface by creating
        an instance of the `Implements` and `Subscribes` classes, passing the
        necessary parameters.

        Args:
            handler (Callable[Any, Any]): Responsible for handling incoming messages
                from the IceConnector.

        """
        self.implements = Implements(self.ice_connector, handler)
        self.subscribes = Subscribes(self.ice_connector, self.topic_manager, handler)

    def get_proxies_map(self):
        """
        Aggregates proxies maps from parent and child classes, returning a unified
        map for further use.

        Returns:
            Dict[str,int]: A dictionary containing the proxies for both required
            and published APIs.

        """
        result = {}
        result.update(self.requires.get_proxies_map())
        result.update(self.publishes.get_proxies_map())
        return result

    def destroy(self):
        """
        Terminates any open connections and destroys the IceConnector object, which
        is a critical component for establishing and managing interfaces.

        """
        if self.ice_connector:
            self.ice_connector.destroy()




