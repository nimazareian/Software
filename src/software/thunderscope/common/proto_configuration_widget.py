from PyQt6.QtCore import QThread, pyqtSignal
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph import parametertree
from google.protobuf.json_format import MessageToDict
from thefuzz import fuzz
from proto.import_all_protos import *
import time


class ProtoConfigurationWidget(QWidget):

    """Creates a searchable parameter widget that can take any protobuf,
    and convert it into a pyqtgraph ParameterTree. This will allow users
    to modify the values.

    """

    def __init__(
        self, proto_to_configure, on_change_callback, search_filter_threshold=60,
    ):
        """Create a parameter widget given a protobuf

        NOTE: This class handles the ParameterRangeOptions

        :param proto_to_configure: The protobuf we would like to generate
                                   a parameter tree for. This should be 
                                   populated with the default values
        :param on_change_callback: The callback to trigger on change
                                    args: name, updated_value, updated_proto
        :param search_filter_threshold: How close should the search query be?
                        100 is an exact match (not ideal), 0 lets everything through

        """
        QWidget.__init__(self)
        layout = QVBoxLayout()
        self.setLayout(layout)
        print("1")

        self.on_change_callback = on_change_callback
        self.proto_to_configure = proto_to_configure

        self.search_worker = SearchConfigurationWorker(proto_to_configure)
        self.search_worker.finished.connect(self.__handle_search_finished)
        print("2")


        # Create ParameterGroup from Protobuf
        self.search_worker.update_search_term("")  # RISE OF CRASH TODO: FIX THIS
        # self.param_group = parametertree.Parameter.create(
        #     name="params",
        #     type="group",
        #     children=self.config_proto_to_param_dict(self.proto_to_configure, None),
        # )

        print("3")
        # Create ParameterTree
        self.param_tree = parametertree.ParameterTree(showHeader=False)
        # self.param_tree.setParameters(self.param_group, showTop=False)
        # self.param_group.sigTreeStateChanged.connect(self.__handle_parameter_changed)
        self.param_tree.setAlternatingRowColors(False)

        # Create search query bar
        self.search_query = QLineEdit()
        self.search_query.textChanged.connect(self.__handle_search_query_changed)
        self.search_filter_threshold = search_filter_threshold
        print("4")

        layout.addWidget(self.search_query)
        layout.addWidget(self.param_tree)

    def __handle_search_finished(self, search_results):
        """Handles the search results by reconfiguring the parameter tree with
        the results.
        """
        self.param_group = parametertree.Parameter.create(
            name="params",
            type="group",
            children=search_results,
        )
        self.param_tree.setParameters(self.param_group, showTop=False)
        self.param_group.sigTreeStateChanged.connect(self.__handle_parameter_changed)

    def __handle_search_query_changed(self, search_term):
        """Given a new search term, reconfigure the parameter tree with parameters
        that match the term.

        NOTE: Messages are not searchable, only fields are searchable/filtered

        :param search_term: The term to filter the parameter tree by
        """

        self.search_worker.update_search_term(search_term)
        #
        # self.param_group = parametertree.Parameter.create(
        #     name="params",
        #     type="group",
        #     children=self.config_proto_to_param_dict(
        #         self.proto_to_configure, search_term
        #     ),
        # )
        # self.param_tree.setParameters(self.param_group, showTop=False)
        # self.param_group.sigTreeStateChanged.connect(self.__handle_parameter_changed)

    def __handle_parameter_changed(self, param, changes):
        """Handles the parameter change by triggering the provided callback

        :param param: The paramaeter that changed
        :param changes: The changes

        """
        for param, change, data in changes:
            path = self.param_group.childPath(param)

            if path is not None:
                child_name = ".".join(path)
            else:
                child_name = param.name()

            # We need to set the updated value, but its hard to differentiate
            # between strings and enums. So we need to try setting the data
            # as a enum first and then as a string. If both raise, then we
            # raise to the main thread because the value wasn't updated.
            #
            # The other types will work with either
            try:
                exec(f"self.proto_to_configure.{child_name} = {data}")
            except (TypeError, NameError):
                exec(f"self.proto_to_configure.{child_name} = data")

            self.on_change_callback(child_name, data, self.proto_to_configure)

    @staticmethod
    def __create_int_parameter(key, value, descriptor):
        """Converts an int field of a proto to a SliderParameter with
        the min/max bounds set according to the provided ParameterRangeOptions

        min/vax options.

        :param key: The name of the parameter
        :param value: The default value
        :param descriptor: The proto descriptor

        """

        # Extract the options from the descriptor, and store it
        # in the dictionary.
        options = MessageToDict(
            descriptor.GetOptions(), preserving_proto_field_name=True
        )

        try:
            min_max = options["[TbotsProto.bounds]"]
        except KeyError:
            raise KeyError("{} missing ParameterRangeOptions".format(key))

        return {
            "name": key,
            "type": "slider",
            "value": value,
            "default": value,
            "limits": (int(min_max["min_int_value"]), int(min_max["max_int_value"])),
            "step": 1,
        }

    @staticmethod
    def __create_double_parameter(key, value, descriptor):
        """Converts a double field of a proto to a SliderParameter with
        the min/max bounds set according to the provided ParameterRangeOptions
        min/vax options.

        :param key: The name of the parameter
        :param value: The default value
        :param descriptor: The proto descriptor

        """

        # Extract the options from the descriptor, and store it
        # in the dictionary.
        options = MessageToDict(
            descriptor.GetOptions(), preserving_proto_field_name=True
        )

        try:
            min_max = options["[TbotsProto.bounds]"]
        except KeyError:
            raise KeyError("{} missing ParameterRangeOptions".format(key))

        return {
            "name": key,
            "type": "slider",
            "value": value,
            "default": value,
            "limits": (min_max["min_double_value"], min_max["max_double_value"],),
            "step": 0.01,
        }

    @staticmethod
    def __create_enum_parameter(key, value, descriptor):
        """Converts an enum field in a protobuf to a ListParameter. Uses
        the options to lookup all possible enum values and provides them
        as a dropdown option.

        :param key: The name of the parameter
        :param value: The default value
        :param descriptor: The proto descriptor

        """
        options = []

        for enum_desc in descriptor.enum_type.values:
            options.append(enum_desc.name)

        # The list index is indexed from 1
        current_enum_index = value - 1

        return parametertree.parameterTypes.ListParameter(
            name=key,
            default=None,
            value=descriptor.enum_type.values[current_enum_index].name,
            limits=options,
        )

    @staticmethod
    def __create_bool_parameter(key, value, _):
        """Convert a bool field in proto to a BoolParameter

        :param key: The name of the parameter
        :param value: The default value
        :param _: The proto descriptor, unused for bool

        """
        return {"name": key, "type": "bool", "value": value}

    @staticmethod
    def __create_string_parameter(key, value, descriptor):
        """Convert a string field in proto to a StrParameter

        :param key: The name of the parameter
        :param value: The default value
        :param descriptor: The proto descriptor

        """
        return {"name": key, "type": "text", "value": " "}

    # def config_proto_to_param_dict(
    #     self, message, search_term=None, current_attr=None,
    # ):
    #     """Converts a protobuf to a pyqtgraph parameter tree dictionary
    #     that can loaded directly into a ParameterTree
    #
    #     https://pyqtgraph.readthedocs.io/en/latest/parametertree/index.html
    #
    #     :param message: The message to convert to a dictionary
    #     :param search_term: The search filter
    #     :param current_attr: Which attr we are currently on to set
    #
    #     """
    #     start_time = time.time()
    #
    #     message_dict = {}
    #     field_list = []
    #
    #     if not current_attr:
    #         current_attr = "self.proto_to_configure"
    #
    #     for descriptor in message.DESCRIPTOR.fields:
    #
    #         key = descriptor.name
    #         value = getattr(message, descriptor.name)
    #
    #         if search_term and descriptor.type != descriptor.TYPE_MESSAGE:
    #             if fuzz.partial_ratio(search_term, key) < self.search_filter_threshold:
    #                 continue
    #
    #         if descriptor.type == descriptor.TYPE_MESSAGE:
    #             field_list.append(
    #                 {
    #                     "name": key,
    #                     "type": "group",
    #                     "children": self.config_proto_to_param_dict(
    #                         value, search_term, f"{current_attr}.{key}",
    #                     ),
    #                 }
    #             )
    #
    #         elif descriptor.type == descriptor.TYPE_BOOL:
    #             field_list.append(self.__create_bool_parameter(key, value, descriptor))
    #
    #         elif descriptor.type == descriptor.TYPE_ENUM:
    #             field_list.append(self.__create_enum_parameter(key, value, descriptor))
    #
    #         elif descriptor.type == descriptor.TYPE_STRING:
    #             field_list.append(
    #                 self.__create_string_parameter(key, value, descriptor)
    #             )
    #
    #         elif descriptor.type == descriptor.TYPE_DOUBLE:
    #             field_list.append(
    #                 self.__create_double_parameter(key, value, descriptor)
    #             )
    #
    #         elif descriptor.type in [descriptor.TYPE_INT32, descriptor.TYPE_INT64]:
    #             field_list.append(self.__create_int_parameter(key, value, descriptor))
    #
    #         else:
    #             raise NotImplementedError(
    #                 "Unsupported type {} in parameter config".format(descriptor.type)
    #             )
    #
    #         # Protobuf doesn't set the default values by default, and won't let
    #         # us serialize the message if all the required fields are not set (even
    #         # if they have a default). So lets just set the default as the value
    #         if descriptor.type != descriptor.TYPE_MESSAGE:
    #             if descriptor.type == descriptor.TYPE_STRING:
    #                 exec(f"{current_attr}.{key} = '{value}'")
    #             else:
    #                 exec(f"{current_attr}.{key} = {value}")
    #
    #     end_time = time.time()
    #     print(f"Time to convert proto to dict: {end_time - start_time}")
    #
    #     if field_list:
    #         return field_list
    #
    #     return message_dict


class SearchConfigurationWorker(QThread):
    finished = pyqtSignal(list)

    def __init__(self, proto_to_configure):
        super(SearchConfigurationWorker, self).__init__(None)
        self.search_result = []
        self.search_term = None
        self.proto_to_configure = proto_to_configure

    def config_proto_to_param_dict(self, message, current_attr=None):
        field_list = []

        if not current_attr:
            current_attr = "self.proto_to_configure"

        for descriptor in message.DESCRIPTOR.fields:

            key = descriptor.name
            value = getattr(message, descriptor.name)

            if self.search_term and descriptor.type != descriptor.TYPE_MESSAGE:
                if fuzz.partial_ratio(self.search_term, key) < 60:  # TODO: make this configurable
                    continue

            if descriptor.type == descriptor.TYPE_MESSAGE:
                field_list.append(
                    {
                        "name": key,
                        "type": "group",
                        "children": self.config_proto_to_param_dict(
                            value, f"{current_attr}.{key}",
                        ),
                    }
                )

            elif descriptor.type == descriptor.TYPE_BOOL:
                field_list.append(self.__create_bool_parameter(key, value, descriptor))

            elif descriptor.type == descriptor.TYPE_ENUM:
                field_list.append(self.__create_enum_parameter(key, value, descriptor))

            elif descriptor.type == descriptor.TYPE_STRING:
                field_list.append(
                    self.__create_string_parameter(key, value, descriptor)
                )

            elif descriptor.type == descriptor.TYPE_DOUBLE:
                field_list.append(
                    self.__create_double_parameter(key, value, descriptor)
                )

            elif descriptor.type in [descriptor.TYPE_INT32, descriptor.TYPE_INT64]:
                field_list.append(self.__create_int_parameter(key, value, descriptor))

            else:
                raise NotImplementedError(
                    f"Unsupported type {descriptor.type} in parameter config"
                )

            # Protobuf doesn't set the default values by default, and won't let
            # us serialize the message if all the required fields are not set (even
            # if they have a default). So lets just set the default as the value
            if descriptor.type != descriptor.TYPE_MESSAGE:
                if descriptor.type == descriptor.TYPE_STRING:
                    exec(f"{current_attr}.{key} = '{value}'")
                else:
                    exec(f"{current_attr}.{key} = {value}")

        if field_list:
            self.finished.emit(field_list)
        else:
            self.finished.emit({})

    def run(self):
        self.config_proto_to_param_dict(self.proto_to_configure)

    def update_search_term(self, new_search_term):
        self.search_term = new_search_term
        self.start()

    @staticmethod
    def __create_int_parameter(key, value, descriptor):
        """Converts an int field of a proto to a SliderParameter with
        the min/max bounds set according to the provided ParameterRangeOptions

        min/vax options.

        :param key: The name of the parameter
        :param value: The default value
        :param descriptor: The proto descriptor

        """

        # Extract the options from the descriptor, and store it
        # in the dictionary.
        options = MessageToDict(
            descriptor.GetOptions(), preserving_proto_field_name=True
        )

        try:
            min_max = options["[TbotsProto.bounds]"]
        except KeyError:
            raise KeyError("{} missing ParameterRangeOptions".format(key))

        return {
            "name": key,
            "type": "slider",
            "value": value,
            "default": value,
            "limits": (int(min_max["min_int_value"]), int(min_max["max_int_value"])),
            "step": 1,
        }

    @staticmethod
    def __create_double_parameter(key, value, descriptor):
        """Converts a double field of a proto to a SliderParameter with
        the min/max bounds set according to the provided ParameterRangeOptions
        min/vax options.

        :param key: The name of the parameter
        :param value: The default value
        :param descriptor: The proto descriptor

        """

        # Extract the options from the descriptor, and store it
        # in the dictionary.
        options = MessageToDict(
            descriptor.GetOptions(), preserving_proto_field_name=True
        )

        try:
            min_max = options["[TbotsProto.bounds]"]
        except KeyError:
            raise KeyError("{} missing ParameterRangeOptions".format(key))

        return {
            "name": key,
            "type": "slider",
            "value": value,
            "default": value,
            "limits": (min_max["min_double_value"], min_max["max_double_value"],),
            "step": 0.01,
        }

    @staticmethod
    def __create_enum_parameter(key, value, descriptor):
        """Converts an enum field in a protobuf to a ListParameter. Uses
        the options to lookup all possible enum values and provides them
        as a dropdown option.

        :param key: The name of the parameter
        :param value: The default value
        :param descriptor: The proto descriptor

        """
        options = []

        for enum_desc in descriptor.enum_type.values:
            options.append(enum_desc.name)

        # The list index is indexed from 1
        current_enum_index = value - 1

        return parametertree.parameterTypes.ListParameter(
            name=key,
            default=None,
            value=descriptor.enum_type.values[current_enum_index].name,
            limits=options,
        )

    @staticmethod
    def __create_bool_parameter(key, value, _):
        """Convert a bool field in proto to a BoolParameter

        :param key: The name of the parameter
        :param value: The default value
        :param _: The proto descriptor, unused for bool

        """
        return {"name": key, "type": "bool", "value": value}

    @staticmethod
    def __create_string_parameter(key, value, descriptor):
        """Convert a string field in proto to a StrParameter

        :param key: The name of the parameter
        :param value: The default value
        :param descriptor: The proto descriptor

        """
        return {"name": key, "type": "text", "value": " "}
