#include <iostream>
#include <vector>
#include <random>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <stack>
#include <chrono>
#include "cnpy.h"

using namespace std;
namespace fs = std::filesystem;

class Grilla {
public:
    int tamX, tamY;
    vector<vector<uint8_t>> mapa;

    Grilla(int x, int y) : tamX(x), tamY(y) {
        mapa = vector<vector<uint8_t>>(tamX, vector<uint8_t>(tamY, 0));
    }

    void colocar_ave(int x, int y) {
        assert(x >= 0 && x < tamX);
        assert(y >= 0 && y < tamY);
        mapa[x][y] = 1;
    }

    void limpiar() {
        for (auto& fila : mapa) {
            fill(fila.begin(), fila.end(), 0);
        }
    }

    int contar_boxes(int div) {
        int boxes_con_aves = 0;
        int celX = tamX / div;
        int celY = tamY / div;

        for (int i = 0; i < div; ++i) {
            for (int j = 0; j < div; ++j) {
                bool hay_ave = false;
                for (int x = 0; x < celX && !hay_ave; ++x) {
                    for (int y = 0; y < celY && !hay_ave; ++y) {
                        if (mapa[i * celX + x][j * celY + y] == 1) {
                            hay_ave = true;
                            ++boxes_con_aves;
                        }
                    }
                }
            }
        }

        return boxes_con_aves;
    }

    void guardar_npy(const string& ruta) {
        vector<uint8_t> plano(tamX * tamY);
        for (int i = 0; i < tamX; ++i) {
            for (int j = 0; j < tamY; ++j) {
                plano[i * tamY + j] = mapa[i][j];
            }
        }
        cnpy::npy_save(ruta, &plano[0], {static_cast<size_t>(tamX), static_cast<size_t>(tamY)}, "w");
    }

    vector<pair<int, int>> contar_colonias(int distancia) {
        vector<pair<int, int>> colonias;
        vector<vector<bool>> visitado(tamX, vector<bool>(tamY, false));

        for (int i = 0; i < tamX; ++i) {
            for (int j = 0; j < tamY; ++j) {
                if (!visitado[i][j] && mapa[i][j] == 1) {
                    int tam = 0;
                    stack<pair<int, int>> pila;
                    pila.push({i, j});
                    visitado[i][j] = true;

                    while (!pila.empty()) {
                        auto [x, y] = pila.top(); pila.pop();
                        ++tam;
                        for (int dx = -distancia; dx <= distancia; ++dx) {
                            for (int dy = -distancia; dy <= distancia; ++dy) {
                                if (dx != 0 || dy != 0) {
                                    int nx = (x + dx + tamX) % tamX;
                                    int ny = (y + dy + tamY) % tamY;
                                    if (!visitado[nx][ny] && mapa[nx][ny] == 1) {
                                        visitado[nx][ny] = true;
                                        pila.push({nx, ny});
                                    }
                                }
                            }
                        }
                    }
                    colonias.emplace_back(tam, 1);
                }
            }
        }

        return colonias;
    }
};

class Ave {
public:
    int x, y;
    int rango_vision;

    Ave(Grilla& g, mt19937& gen, int rango, bool es_semilla, double P_Base) : rango_vision(rango) {
        uniform_int_distribution<> distX(0, g.tamX - 1);
        uniform_int_distribution<> distY(0, g.tamY - 1);
        uniform_real_distribution<> distR(0.0, 1.0);

        double r;
        do {
            x = distX(gen);
            y = distY(gen);
            r = distR(gen);
        } while (
            g.mapa[x][y] == 1 || 
            (!es_semilla && r > max(fn_atraccion(g), P_Base))
        );

        g.colocar_ave(x, y);
    }

    int contar_vecinos(const Grilla& g, int d) {
        int suma = 0;
        for (int dx = -d; dx <= d; ++dx) {
            for (int dy = -d; dy <= d; ++dy) {
                if (abs(dx) == d || abs(dy) == d) {
                    int nx = (x + dx + g.tamX) % g.tamX;
                    int ny = (y + dy + g.tamY) % g.tamY;
                    suma += g.mapa[nx][ny];
                }
            }
        }
        return suma;
    }

    double fn_atraccion(const Grilla& g) {
        double C = 0.0, suma = 0.0;
        for (int i = 1; i <= rango_vision; ++i) {
            int sitios = 8 * i;
            C += 1.0 / i;
            suma += (contar_vecinos(g, i) / static_cast<double>(sitios)) / i;
        }
        return C * suma;
    }

    void recolocar(Grilla& g, mt19937& gen, double P_Base) {
        uniform_int_distribution<> distX(0, g.tamX - 1);
        uniform_int_distribution<> distY(0, g.tamY - 1);
        uniform_real_distribution<> distR(0.0, 1.0);

        g.mapa[x][y] = 0;

        int nx, ny;
        double r;
        do {
            nx = distX(gen);
            ny = distY(gen);
            r = distR(gen);
        } while (g.mapa[nx][ny] == 1 || r > max(fn_atraccion(g), P_Base));

        x = nx;
        y = ny;
        g.colocar_ave(x, y);
    }
};

void crear_carpeta(const string& path) {
    if (!fs::exists(path)) {
        fs::create_directories(path);
    }
}


vector<pair<double, double>> encontrar_bines(const vector<pair<int, int>>& colonias) {
    vector<int> tamanios;
    for (const auto& [tam, freq] : colonias) {
        for (int i = 0; i < freq; ++i) {
            tamanios.push_back(tam);
        }
    }
    

    int max_tam = *max_element(tamanios.begin(), tamanios.end());
    int base = 2;
    vector<pair<double, double>> bines_finales;

    bool sin_bines_vacios = false;
    vector<int> vec_bines_count;

    while (!sin_bines_vacios && base <= max_tam) {
        vec_bines_count.clear();
        int pot_max = 0;
        while (pow(base, pot_max) < max_tam) {
            ++pot_max;
        }

        vec_bines_count.resize(pow(base, pot_max) <= max_tam ? pot_max + 1 : pot_max, 0);

        for (int tam : tamanios) {
            for (int i = 0; i < vec_bines_count.size(); ++i) {
                int bin_min = pow(base, i);
                int bin_max = pow(base, i + 1);
                if (tam >= bin_min && tam < bin_max) {
                    vec_bines_count[i]++;
                    break;
                }
            }
        }

        sin_bines_vacios = all_of(vec_bines_count.begin(), vec_bines_count.end(),
                                  [](int count) { return count > 0; });

        if (!sin_bines_vacios) {
            ++base;
        }
    }

    if (!sin_bines_vacios) {
        cerr << "Advertencia: No se pudo evitar bines vacíos. Se usa base = " << base - 1 << " con bines vacíos.\n";
    }

    // Construir bines finales con la última base utilizada
    for (int i = 0; i < vec_bines_count.size(); ++i) {
        int bin_min = pow(base - (!sin_bines_vacios), i);
        int bin_max = pow(base - (!sin_bines_vacios), i + 1);
        if (vec_bines_count[i] > 0) {
            double centro = pow(10.0, (log10(bin_min) + log10(bin_max-1.0)) / 2.0);
            double densidad = static_cast<double>(vec_bines_count[i]) / ((bin_max - bin_min) * tamanios.size());
            bines_finales.emplace_back(centro, densidad);
        }
    }

    return bines_finales;
}

int main(int argc, char* argv[]) {
    // Capturar el tiempo de inicio
    auto start = chrono::high_resolution_clock::now();
    try {
        // Inicialización de las variables...
        int pot_tam_grilla = 10;
        float proporcion_aves = 0.01;
        float aves_semillas = 0.05;
        int nro_tandas = 3;
        int rango_vision = 3;
        int distancia_colonia = 1;
        double Prob_Base=0.0001;
        string nombre_gral = "Default_Carpeta";
        string nombre_chiquito = "Subcarpeta";

        // Parseo de los argumentos...
        if (argc == 10) {
            pot_tam_grilla = stoi(argv[1]);
            proporcion_aves = stof(argv[2]);
            aves_semillas = stof(argv[3]);
            nro_tandas = stoi(argv[4]);
            rango_vision = stoi(argv[5]);
            distancia_colonia= stoi(argv[6]);
            Prob_Base= stod(argv[7]);
            nombre_gral = argv[8];
            nombre_chiquito = argv[9];
        } else {
            cout << "Usando valores por defecto (modo VSCode o sin argumentos)" << endl;
        }

        // Impresión de los parámetros...
        cout << "Ejecutando con: pot_tam_grilla: " << pot_tam_grilla
            << ", proporcion_aves: " << proporcion_aves
            << ", aves_semillas: " << aves_semillas
            << ", nro_tandas: " << nro_tandas
            << ", rango_vision: " << rango_vision
            << ", distancia_colonia: "<< distancia_colonia
            << ", Prob_Base: " << Prob_Base
            << ", nombre_gral: " << nombre_gral
            << ", nombre_chiquito: " << nombre_chiquito << endl;

        int tam = 1 << pot_tam_grilla;
        int nro_aves = static_cast<int>(tam * tam * proporcion_aves);

        Grilla grilla(tam, tam);

        random_device rd;
        mt19937 gen(rd());

        vector<Ave> aves;
        int cant_semillas = static_cast<int>(nro_aves * aves_semillas);
        for (int i = 0; i < nro_aves; ++i) {
            bool es_semilla = (i < cant_semillas);
            aves.emplace_back(grilla, gen, rango_vision, es_semilla, Prob_Base);
        }

        string carpeta = nombre_gral + "/" + nombre_chiquito;
        crear_carpeta(carpeta);

        // Tanda 0 (inicial)
        grilla.guardar_npy(carpeta + "/Mapa0.npy");

        // Contar las cajas (counting box) para la tanda 0
        vector<pair<int, int>> counting_box_results_0;
        for (int div = 2; div <= tam; div *= 2) {
            int resultado = grilla.contar_boxes(div);
            counting_box_results_0.emplace_back(tam / div, resultado);
        }

        // Guardar el resultado de counting_box para la tanda 0
        ofstream box_file_0(carpeta + "/Datos_Box_0.csv");
        for (const auto& [tam_celda, cant] : counting_box_results_0) {
            box_file_0 << tam_celda << "\t" << cant << "\n";
        }
        box_file_0.close();

        // Contar colonias y generar Vec_fx para la tanda 0
        auto colonias_0 = grilla.contar_colonias(distancia_colonia);

        bool todas_uno_0 = all_of(colonias_0.begin(), colonias_0.end(),
                                [](const pair<int, int>& c) { return c.first == 1; });

        if (colonias_0.size() <= 1 || todas_uno_0) {
            cout << "Se omitió el cálculo de Vec_fx para tanda 0: solo una colonia o todas con tamaño 1." << endl;
        } else {
            auto bines_0 = encontrar_bines(colonias_0);
            ofstream freq_file_0(carpeta + "/Vec_fx_0.csv");
            for (const auto& [x, y] : bines_0) {
                freq_file_0 << x << "\t" << y << "\n";
            }
            freq_file_0.close();
        }

        // Simulaciones para las demás tandas (1 a nro_tandas)
        for (int t = 1; t <= nro_tandas; ++t) {
            cout << "Tanda: " << t << endl;
        
            vector<Ave> nuevas_aves;
        
            for (size_t i = 0; i < aves.size(); ++i) {
                // Borrar ave vieja
                grilla.mapa[aves[i].x][aves[i].y] = 0;
        
                // Crear nueva ave usando función de atracción
                nuevas_aves.emplace_back(grilla, gen, rango_vision, false, Prob_Base);
            }
        
            // Reemplazar las aves viejas por las nuevas
            aves = move(nuevas_aves);
        
            // Guardar mapa actualizado
            grilla.guardar_npy(carpeta + "/Mapa" + to_string(t) + ".npy");
                
            // Contar las cajas (counting box) para la tanda t
            vector<pair<int, int>> counting_box_results;
            for (int div = 2; div <= tam; div *= 2) {
                int resultado = grilla.contar_boxes(div);
                counting_box_results.emplace_back(tam / div, resultado);
            }

            // Guardar el resultado de counting_box para la tanda t
            ofstream box_file(carpeta + "/Datos_Box_" + to_string(t) + ".csv");
            for (const auto& [tam_celda, cant] : counting_box_results) {
                box_file << tam_celda << "\t" << cant << "\n";
            }
            box_file.close();

            // Contar colonias y generar Vec_fx para la tanda t
            auto colonias = grilla.contar_colonias(distancia_colonia);

            bool todas_uno = all_of(colonias.begin(), colonias.end(),
                                    [](const pair<int, int>& c) { return c.first == 1; });

            if (colonias.size() <= 1 || todas_uno) {
                cout << "Se omitió el cálculo de Vec_fx para tanda " << t << ": solo una colonia o todas con tamaño 1." << endl;
            } else {
                auto bines = encontrar_bines(colonias);
                ofstream freq_file(carpeta + "/Vec_fx_" + to_string(t) + ".csv");
                for (const auto& [x, y] : bines) {
                    freq_file << x << "\t" << y << "\n";
                }
                freq_file.close();
            }
        }

        cout << "Simulación completada." << endl;
    }catch (const exception& e) {
        cerr << "Excepción atrapada en main: " << e.what() << endl;
    }
    // Capturar el tiempo de fin
    auto end = chrono::high_resolution_clock::now();

    // Calcular la duración en segundos
    auto duration = chrono::duration_cast<chrono::seconds>(end - start);

    // Convertir la duración a minutos y segundos
    int minutes = duration.count() / 60;
    int seconds = duration.count() % 60;

    // Mostrar el tiempo transcurrido en minutos y segundos
    cout << "Tiempo de ejecución: " << minutes << " minutos y " << seconds << " segundos" << endl;

    return 0;
}
