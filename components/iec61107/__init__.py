import esphome.config_validation as cv

CODEOWNERS = ["@latonita"]

CONFIG_SCHEMA = cv.invalid(
    "Component 'iec61107' from 'latonita/esphome-iec61107-meter' got renamed.\n"
    "Please use the component 'energomera_iec' from 'github://latonita/esphome-energomera-iec'.\n"
    "Visit https://github.com/latonita/esphome-energomera-iec to see updated config options.\n"
)
