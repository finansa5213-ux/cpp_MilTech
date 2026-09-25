// Сухий прогін арбітра й багаторівневого захисту: без заліза, без сокетів,
// з керованим годинником. Той самий сценарій, що на живому стенді 18.09.
#include "uav/Arbiter.hpp"
#include "uav/Failsafe.hpp"
#include "uav/LinkMonitor.hpp"
#include "tests/Check.hpp"

using namespace uav;

namespace {

/// Керований годинник: тест не спить, він рухає час.
struct FakeClock {
    TimePoint t = TimePoint{} + std::chrono::seconds(1000);
    void      advance(double sec) {
        t += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(sec));
    }
};

} // namespace

int main() {
    Arbiter::Params p;                 // пороги за умовчанням = виміряні
    Arbiter   arb(p);
    Failsafe  fs(Failsafe::Params{5.0, /*commandRtl=*/true});
    FakeClock clk;

    auto step = [&](double sec, bool aAlive, bool bAlive, double fullMs, bool haveRtt = true) {
        Arbiter::Decision last;
        const double dt = 0.05;
        for (double e = 0; e < sec; e += dt) {
            clk.advance(dt);
            Arbiter::Inputs in;
            in.aAlive = aAlive;
            in.bAlive = bAlive;
            in.armed  = true;
            if (haveRtt) in.fullPathMs = fullMs;
            const auto d = arb.update(clk.t, in);
            if (d.changed) last = d;
            fs.update(clk.t, true, aAlive || bAlive, arb.active(), /*navUsable=*/true);
        }
        return last;
    };

    // --- сценарій, знятий на стенді ------------------------------------
    step(1.0, true, true, 140);
    CHECK(arb.active() == Channel::B, "обидва канали живі, B у нормі -> B");

    step(1.0, true, false, 140);
    CHECK(arb.active() == Channel::A, "B зник -> рівень 1, перехід на A");

    step(5.0, true, true, 140);
    CHECK(arb.active() == Channel::A, "B повернувся, витримка 10 с ще тримає");

    step(6.0, true, true, 140);
    CHECK(arb.active() == Channel::B, "після 10 с стабільності -> назад на B");

    step(1.0, true, true, 300);
    CHECK(arb.active() == Channel::A, "B живий, але шлях 300 мс > 200 -> на A");

    step(5.0, true, true, 180);
    CHECK(arb.active() == Channel::A, "180 мс у гістерезисній смузі - не повертаємось");

    step(12.0, true, true, 140);
    CHECK(arb.active() == Channel::B, "140 мс нижче 170 -> повернення дозволене");

    // --- рівні 2 і 3 ---------------------------------------------------
    step(4.0, false, false, 0, false);
    CHECK(arb.active() == Channel::Failsafe, "обидва мовчать -> режим відмови");
    CHECK(fs.level() == Failsafe::Level::BothLost || fs.level() == Failsafe::Level::SelfReturn,
          "рівень захисту піднявся щонайменше до 2");

    bool sawRtl = false;
    for (int i = 0; i < 200; ++i) {
        clk.advance(0.05);
        const auto a = fs.update(clk.t, true, false, arb.active(), /*navUsable=*/true);
        if (a.sendRtl) sawRtl = true;
    }
    CHECK(sawRtl, "рівень 3: команду повернення сформовано");
    CHECK(fs.level() == Failsafe::Level::SelfReturn, "рівень захисту = 3");

    step(1.0, false, true, 140);
    CHECK(arb.active() == Channel::B, "зв'язок відновлено -> вихід із режиму відмови");

    // --- ручне керування -----------------------------------------------
    arb.requestManual(Channel::A, clk.t);
    CHECK(arb.active() == Channel::A, "команда оператора: перехід на A");
    step(5.0, true, true, 100);
    CHECK(arb.active() == Channel::A, "у ручному режимі автоматика мовчить");
    arb.requestAuto();
    step(12.0, true, true, 100);
    CHECK(arb.active() == Channel::B, "після AUTO автоматика повертає на B");

    // --- лічильники: загальний і за вікно --------------------------------
    {
        const unsigned total = arb.switches();
        CHECK(total > 0, "загальний лічильник рахує від запуску");
        CHECK_EQ(arb.switchesInWindow(), total, "до першого звіту обидва збігаються");

        arb.resetWindow();
        CHECK_EQ(arb.switchesInWindow(), 0u, "звіт обнулив віконний лічильник");
        CHECK_EQ(arb.switches(), total, "загальний лічильник звіт не чіпає");

        step(12.0, true, true, 100);   // спокійне вікно: жодного перемикання
        CHECK_EQ(arb.switchesInWindow(), 0u, "спокійне вікно видно одразу, без порівняння рядків");

        step(1.0, false, true, 100);   // A зник - на вибір каналу це не впливає
        step(1.5, true, false, 100);   // B зник - ось тепер перемикання
        CHECK_EQ(arb.switchesInWindow(), 1u, "неспокійне вікно порахувало рівно одне");
        CHECK_EQ(arb.switches(), total + 1, "загальний лічильник просунувся разом із ним");
    }

    // --- захист не озброєний до першого контакту -------------------------
    {
        Arbiter fresh(p);
        Arbiter::Inputs in;           // жодного контакту з землею: armed = false
        for (int i = 0; i < 200; ++i) { clk.advance(0.05); fresh.update(clk.t, in); }
        CHECK(fresh.active() == Channel::B && fresh.switches() == 0,
              "до першого контакту з землею перемикань немає");
    }

    // --- захист не озброєний до першого контакту з землею -------------------
    // Знайдено першим же запуском на живому залізі: на увімкненні живлення
    // обидва канали природно мовчать, і без цієї умови маршрутизатор оголошував
    // рівень 2 у першу ж мить, а через п'ять секунд скомандував би повернення
    // апарату, який ще ні з ким не з'єднався.
    {
        Failsafe  boot(Failsafe::Params{5.0, /*commandRtl=*/true});
        FakeClock c;
        bool      anyRtl = false, anyLevel = false;
        for (int i = 0; i < 400; ++i) {          // 20 с тиші від запуску
            c.advance(0.05);
            const auto a = boot.update(c.t, /*armed=*/false, /*anyAlive=*/false, Channel::B,
                                       /*navUsable=*/true);
            if (a.sendRtl) anyRtl = true;
            if (a.level != Failsafe::Level::Normal) anyLevel = true;
        }
        CHECK(!anyLevel, "до першого контакту рівень захисту лишається нульовим");
        CHECK(!anyRtl, "до першого контакту команда повернення не формується");

        c.advance(0.05);
        boot.update(c.t, /*armed=*/true, /*anyAlive=*/false, Channel::B, /*navUsable=*/true);
        CHECK(boot.level() == Failsafe::Level::BothLost,
              "після першого контакту захист озброюється");
    }

    // --- рівень 3 не командує повернення без придатної навігації -----------
    // Підстава - вимірювання 20.09.2026: під час підміни GNSS оцінювач зняв
    // ознаку дійсної абсолютної горизонтальної координати, а ref_lat/ref_lon
    // дорівнювали nan. Повернення в такому стані нездійсненне: немає звідки
    // відлічувати шлях додому.
    {
        Failsafe  nav(Failsafe::Params{5.0, /*commandRtl=*/true, /*requireNav=*/true});
        FakeClock c;
        bool      anyRtl = false, blocked = false;

        c.advance(0.05);
        nav.update(c.t, /*armed=*/true, /*anyAlive=*/true, Channel::B, /*navUsable=*/false);
        for (int i = 0; i < 400; ++i) {           // 20 с тиші обох каналів
            c.advance(0.05);
            const auto a = nav.update(c.t, true, /*anyAlive=*/false, Channel::Failsafe,
                                      /*navUsable=*/false);
            if (a.sendRtl)    anyRtl  = true;
            if (a.navBlocked) blocked = true;
        }
        CHECK(!anyRtl, "навігація непридатна: команда повернення не формується");
        CHECK(blocked, "непридатну навігацію оголошено окремою ознакою");
        CHECK(nav.level() == Failsafe::Level::SelfReturn,
              "час рівня 3 настав, хоч команди й немає");

        // Та сама витримка, але з придатною навігацією, - команда має бути.
        Failsafe  ok(Failsafe::Params{5.0, /*commandRtl=*/true, /*requireNav=*/true});
        FakeClock c2;
        bool      okRtl = false, okBlocked = false;
        c2.advance(0.05);
        ok.update(c2.t, true, /*anyAlive=*/true, Channel::B, /*navUsable=*/true);
        for (int i = 0; i < 400; ++i) {
            c2.advance(0.05);
            const auto a = ok.update(c2.t, true, false, Channel::Failsafe, /*navUsable=*/true);
            if (a.sendRtl)    okRtl     = true;
            if (a.navBlocked) okBlocked = true;
        }
        CHECK(okRtl, "навігація придатна: команду повернення сформовано");
        CHECK(!okBlocked, "придатна навігація ознаки блокування не піднімає");

        // Вимкнена перевірка має повертати попередню поведінку: це запобіжник
        // на випадок контролера, який ESTIMATOR_STATUS не передає взагалі.
        Failsafe  off(Failsafe::Params{5.0, /*commandRtl=*/true, /*requireNav=*/false});
        FakeClock c3;
        bool      offRtl = false;
        c3.advance(0.05);
        off.update(c3.t, true, /*anyAlive=*/true, Channel::B, /*navUsable=*/false);
        for (int i = 0; i < 400; ++i) {
            c3.advance(0.05);
            if (off.update(c3.t, true, false, Channel::Failsafe, /*navUsable=*/false).sendRtl)
                offRtl = true;
        }
        CHECK(offRtl, "require_nav_for_rtl=0 повертає поведінку без перевірки");
    }

    // --- монітор: прострочені виміри недійсні -----------------------------
    {
        LinkMonitor m("B", 3.0, 5.0);
        m.onGroundFrame(clk.t, 21);
        m.onRttSample(65.0, clk.t);
        CHECK(m.alive(clk.t), "щойно почули землю - канал живий");
        CHECK(m.rttMedianMs(clk.t).has_value(), "свіжий вимір дійсний");
        clk.advance(6.0);
        CHECK(!m.alive(clk.t), "через 6 с тиші канал вважається втраченим");
        CHECK(!m.rttMedianMs(clk.t).has_value(),
              "мертвий канал не звітує про затримку");
    }

    // --- медіана: неповний буфер і стійкість до викиду ---------------------
    {
        LinkMonitor m("B", 3.0, 5.0);
        for (double v : {100.0, 50.0, 200.0}) m.onRttSample(v, clk.t);
        CHECK_NEAR(*m.rttMedianMs(clk.t), 100.0, 1e-9,
                   "три виміри: медіана 100, а не середнє 117");

        // Один спалах 675 мс - саме той найгірший випадок із розділу 4 -
        // не повинен зрушити рішення: медіана його відкидає.
        for (int i = 0; i < 6; ++i) m.onRttSample(70.0, clk.t);
        m.onRttSample(675.0, clk.t);
        CHECK_NEAR(*m.rttMedianMs(clk.t), 70.0, 1e-9,
                   "одиночний викид 675 мс медіану не зрушив");

        LinkMonitor f("B", 3.0, 5.0);
        for (double v : {140.0, 130.0, 200.0, 135.0}) f.onFullPathSample(v, clk.t);
        CHECK_NEAR(*f.fullPathMs(clk.t, 68.0), 140.0, 1e-9,
                   "повний шлях: медіана з чотирьох вимірів");
    }

    return uav::test::summary("Arbiter");
}
