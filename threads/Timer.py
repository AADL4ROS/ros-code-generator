import logging
from pint import UnitRegistry

from threads.AADLThread import AADLThread

import threads.AADLThreadFunctionsSupport as tfs

from datatypes.Type import Void, ROS_TimerEvent, ROS_Timer

from variables.Variable import Variable
from methods.Method import Method

log = logging.getLogger("root")
ureg = UnitRegistry()


class Timer(AADLThread):
    def __init__(self, _system_root, _process, _thread, _associated_class):
        super().__init__(_system_root, _process, _thread, _associated_class)
        log.info("Timer thread {}".format(self.name))

        # Parametri del Timer
        self.source_text_function = None
        self.frequency_in_hz = None
        self.period_in_seconds = None
        self.timerCallback = None

    def populateData(self):
        main_thread = self.associated_class.getMainThread()

        if main_thread is None:
            return False, "Unable to get the Main Thread"

        # Ottengo le informazioni necessarie per i thread di tipo Timer:
        # - Source Text
        # - Period

        thread_function = tfs.getSubprogram(self.thread)
        if thread_function is None:
            return False, "Unable to find the right Subprogram"

        # TRANSFORMATION FRAME

        # Controllo l'uso del Transformation Frame
        self.thread_uses_tf = self.setUsesTransformationFrame()

        # Source Text

        self.source_text_function = self.createSourceTextFileFromSourceText(tfs.getSourceText(thread_function),
                                                                            tfs.getSourceName(thread_function))

        if self.source_text_function is None:
            return False, "Unable to find property Source_Text or Source_Name"

        self.source_text_function.setTF(self.thread_uses_tf)

        # FREQUENCY

        (period, period_unit) = tfs.getPeriod(self.thread)

        if period is None or period_unit is None:
            return False, "Unable to find property Period with relative value and unit"

        # Conversione in secondi della frequenza a partire da qualunque unità di misura
        try:
            period_quantity = ureg("{} {}".format(period, period_unit))
            period_quantity.ito(ureg.second)
            self.frequency_in_hz = 1.0 / period_quantity.magnitude
            self.period_in_seconds = period_quantity.magnitude
        except ValueError:
            return False, "Unable to convert Period in seconds"

        # TIMER
        var_timer = Variable(self.associated_class)
        var_timer.setName("timer_{}".format(self.name))
        var_timer.setType(ROS_Timer(self.associated_class))
        self.associated_class.addInternalVariable(var_timer)

        ######################
        ### TIMER CALLBACK ###
        ######################

        self.timerCallback = Method(self.associated_class)
        self.timerCallback.method_name = "{}_callback".format(self.name)
        self.timerCallback.return_type = Void(self.associated_class)
        self.timerCallback.namespace = self.associated_class.class_name

        input_par = Variable(self.associated_class)
        input_par.setIsParameter()
        input_par.setType(ROS_TimerEvent(self.associated_class))
        input_par.setName("")
        self.timerCallback.addInputParameter(input_par)

        # Aggiungo la chiamata alla funzione custom
        if self.source_text_function:
            code = "{};".format(self.source_text_function.generateInlineCode())
            self.timerCallback.addMiddleCode(code)

        self.associated_class.addPrivateMethod(self.timerCallback)

        main_thread.prepare.addMiddleCode("{} = handle.createTimer(ros::Duration({}), {}, this);"
                                          .format(var_timer.name, self.period_in_seconds,
                                                  self.timerCallback.getThreadPointer()))

        return True, ""
