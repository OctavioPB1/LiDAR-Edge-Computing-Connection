start mosquitto -v -c "C:\Program Files (x86)\mosquitto\mosquitto.conf"
start cmd /k "cd backend && mvn spring-boot:run"
start cmd /k "cd Frontend && ng serve"