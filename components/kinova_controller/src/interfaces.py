import time
import Ice
import IceStorm
from rich.console import Console, Text
console = Console()


Ice.loadSlice("-I ./src/ --all ./src/KinovaArm.ice")
import RoboCompKinovaArm

class TJointSeq(list):
    def __init__(self, iterable=list()):
        super(TJointSeq, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompKinovaArm.TJoint)
        super(TJointSeq, self).append(item)

    def extend(self, iterable):
        """
        Adds an iterable of `RoboCompKinovaArm.TJoint` instances to the sequence
        of `TJointSeq`.

        Args:
            iterable (int): sequence of `RoboCompKinovaArm.TJoint` objects to be
                extended.

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
        Takes an iterable and asserts that all elements are instance of `float`.
        It then extends the `Speeds` class with the elements from the iterable.

        Args:
            iterable (float): list of floating-point values to be added to the
                `Speeds` instance.

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
        Verifies that each element in the iterable is a floating-point number
        before appending it to the Angles class object.

        Args:
            iterable (float): sequence of floating-point numbers to be extended
                by the Angle class through the use of its `super()` method, ensuring
                that all elements within the sequence are instance objects of the
                float type.

        """
        for item in iterable:
            assert isinstance(item, float)
        super(Angles, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, float)
        super(Angles, self).insert(index, item)

setattr(RoboCompKinovaArm, "Angles", Angles)

import kinovaarmI



class Publishes:
    def __init__(self, ice_connector, topic_manager):
        """
        Initializes attributes and relationships between objects, such as
        `ice_connector`, `mprx`, and `topic_manager`.

        Args:
            ice_connector (`object`.): ices connector instance to which the method
                applies.
                
                	* `self.ice_connector`: The initialized instance of the IceConnector
                class, which represents an Intermediate Representation (IR) for a
                C++ ABI-compatible binary module.
            topic_manager (`object`.): topcis_to_be_exported to be exported through
                the Ice connector.
                
                	* `mprx`: A dictionary that contains the multiprocessing event
                loop and other related variables.

        """
        self.ice_connector = ice_connector
        self.mprx={}
        self.topic_manager = topic_manager


    def create_topic(self, topic_name, ice_proxy):
        # Create a proxy to publish a AprilBasedLocalization topic
        """
        Creates a new topic or retrieves an existing one based on its name. It
        returns a publisher proxy for the created or retrieved topic.

        Args:
            topic_name (str): name of a topic to be retrieved, created, or published
                in the function.
            ice_proxy (`IceProxy` object.): iced-based publisher of a topic and
                converts it to an unchecked IcePython proxy for further use.
                
                	* `ice_oneway()`: This method returns a publisher proxy that
                allows the client to send messages to the server without expecting
                any response.
                	* `uncheckedCast(pub)`: This method casts the `publisher_t` object
                to an `ice_proxy_t` object, allowing the client to use the published
                methods on the proxy.

        Returns:
            undefined: a publisher object for the specified topic.
            
            	* `pub`: The topic publisher that is one-way ice.
            	* `proxy`: The ice proxy that unchecked casts the publisher to ice.
            	* `mprx`: A dictionary with the created topic as its key and the ice
            proxy as its value.

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
        self.ice_connector = ice_connector
        self.mprx={}

    def get_proxies_map(self):
        return self.mprx

    def create_proxy(self, property_name, ice_proxy):
        # Remote object connection for
        """
        1) gets a property name (e.g., 'image/32bit') from the input string and
        2) returns a proxy object for that property or throws an exception if it
        fails to connect to the remote object.

        Args:
            property_name (str): name of the property to retrieve from the remote
                object.
            ice_proxy (Ice::Prx instance.): Ice::Internal::UnsafePrx object that
                will be used to wrap the proxy string returned by `stringToProxy()`
                method and turn it into a valid Ice::Object Prx.
                
                	* `uncheckedCast`: This attribute returns an ice proxy object
                that is unchecked and can be cast directly to any interface. (1)
                	* `mprx`: A dictionary containing a property name as the key and
                an ice proxy object as the value, where the property name refers
                to the property of the remote object being accessed. (2)
                
                	In the function, if a property `property_name` is given in the
                input, it is used to access the corresponding ice proxy object
                from the `mprx` dictionary and return `(True, proxy)`. Otherwise,
                an error message is displayed and the function returns `(False, None)`.

        Returns:
            undefined: a tuple of two values: a boolean indicator of whether the
            proxy was created successfully and the created proxy object.
            
            	* `True`: Indicates whether the proxy was created successfully (True)
            or not (False).
            	* `proxy`: The proxy object for the `CameraSimple` service, which can
            be used to communicate with the remote object.
            
            	In general, the `create_proxy` function takes a property name as input
            and returns a tuple containing the result of creating a proxy for the
            specified property and the resulting proxy object.

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
        self.ice_connector = ice_connector
        self.topic_manager = topic_manager

    def create_adapter(self, property_name, interface_handler):
        """
        Creates an Ice transport adapter for a given property name and activates
        it. It also subscribes to a topic with a given name and returns the adapter
        instance.

        Args:
            property_name (str): name of the topic to subscribe to and is used to
                create the appropriate topic in the topic manager and to specify
                the subscription QoS.
            interface_handler (int): icy interface that the created adapter will
                handle.

        Returns:
            undefined: an instance of the `IceContainer` class with a reference
            to an Ice adapter and a publisher.
            
            	* `adapter`: This is an Ice::Adapter object, which represents a
            connection point for communication between different objects and
            processes in the application.
            	* `handler`: This is an Ice::IntfHandler object, which manages the
            interface that is implemented by the adapter.
            	* `proxy`: This is an Ice::OneWayOnly proxy object, which represents
            a one-way message pipe between the client and the server.
            	* `topic_name`: This is a string that represents the name of the topic
            that the adapter listens to.
            	* `subscribe_done`: This is a boolean flag that indicates whether the
            adapter has successfully subscribed to the topic or not.
            	* `qos`: This is an Ice::QoS object, which contains information about
            the quality of service (QoS) settings for the topic subscription.
            	* `activate`: This is a boolean flag that indicates whether the adapter
            has been activated or not.

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
        self.kinovaarm = self.create_adapter("KinovaArm", kinovaarmI.KinovaArmI(default_handler))

    def create_adapter(self, property_name, interface_handler):
        """
        Creates an adapter object for a specified property name using the
        `ice_connector` module and adds an interface handler to it. It also activates
        the adapter object.

        Args:
            property_name (str): identity string for an object to be added to an
                adapter, which is then activated by the `activate()` method call.
            interface_handler (`Identity`.): Java object that defines an interface
                for communicating with the remote endpoint, which is added to the
                ObjectAdapter created by the `self.ice_connector.createObjectAdapter()`
                method.
                
                	* `self.ice_connector`: A reference to an Ice::Connector object.
                It is used in creating objects adapters and managing communications
                between client and server applications built on top of the DICE framework.
                	* `stringToIdentity(property_name.lower())`: This method takes a
                string argument and returns an Ice::Identity object. The identity
                can be any user-defined or predefined name used to identify an
                interface, class, or function within an Ice framework. It converts
                the input string to an appropriate Ice identity using lowercase
                lettering as required by most Ice API functions and objects.
                	* `add(interface_handler, self.ice_connector.stringToIdentity(property_name.lower()))`:
                This line adds the provided interface handler to an adapter, along
                with its identity created from property name conversion.

        """
        adapter = self.ice_connector.createObjectAdapter(property_name)
        adapter.add(interface_handler, self.ice_connector.stringToIdentity(property_name.lower()))
        adapter.activate()


class InterfaceManager:
    def __init__(self, ice_config_file):
        # TODO: Make ice connector singleton
        """
        Initialize Ice runtime instance, reads configuration file `ice_config_file`,
        sets up TopicManager if remote connection node (RCNode) is required, and
        sets properties for object.

        Args:
            ice_config_file (str): configuration file for Ice, which is used to
                initialize and configure the Ice client.

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
        Obtains the proxy for the TopicManager service from the Ice connector and
        returns a reference to the TopicManager PRX object after checking its validity.

        Returns:
            undefined: an IceStorm.TopicManagerPrx object, which represents a proxy
            for the Topic Manager service.
            
            	* `obj`: This is an instance of the `TopicManagerPrx` class, which
            represents a proxy for the Topic Manager interface.
            	* `proxy`: This is a string that contains the Proxy identifier for
            the Topic Manager interface.

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
        Updates an existing result object `result` with proxy information from the
        `requires` and `publishes` objects.

        Returns:
            undefined: a dictionary of proxies.
            
            	The result is a dictionary-like object with two keys - `update`. These
            keys represent the proxies map for both requirements and publications,
            respectively. Each key contains a list of tuples, where each tuple
            consists of a proxy URL and its corresponding weight (a non-negative
            integer).

        """
        result = {}
        result.update(self.requires.get_proxies_map())
        result.update(self.publishes.get_proxies_map())
        return result

    def destroy(self):
        if self.ice_connector:
            self.ice_connector.destroy()




